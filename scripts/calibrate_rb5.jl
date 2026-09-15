using LinearAlgebra, DelimitedFiles, Rotations, JuMP, COSMO, TOML

# Reuse the authors' objective and SO(3) constraints. This entry point handles
# the known-scale, single-camera/single-board case; the generalized code remains
# available in experiments/rw_multi_eye_multi_hand.jl.
include(joinpath(@__DIR__, "..", "src", "calibration", "robot_world_costs.jl"))
include(joinpath(@__DIR__, "..", "src", "rotation_sdp_constraints.jl"))

function read_poses(path)
    rows = readdlm(path, ',', Float64)
    size(rows, 2) == 7 || error("$path: expected qw,qx,qy,qz,tx,ty,tz (no header)")
    all(isfinite, rows) || error("$path: non-finite measurements")
    poses = zeros(size(rows, 1), 4, 4)
    poses[:, 4, 4] .= 1
    for (i, row) in enumerate(eachrow(rows))
        abs(norm(row[1:4]) - 1) < 1e-5 || error("$path: non-unit quaternion at row $i")
        poses[i, 1:3, 1:3] = Matrix(QuatRotation(row[1:4]...))
        poses[i, 1:3, 4] = row[5:7]
    end
    return poses
end

function project_rotation(R)
    U, _, V = svd(R)
    return U * Diagonal([1., 1., det(U * V')]) * V'
end

function solve_calibration(A, B; kappa=125.0, translation_sigma_m=0.01, tolerance=1e-6)
    size(A) == size(B) || error("A and B must have the same number of paired poses")
    n = size(A, 1)
    n >= 8 || error("At least 8 poses required; collect 20–40 diverse poses")
    all(isfinite, [kappa, translation_sigma_m, tolerance]) &&
        min(kappa, translation_sigma_m, tolerance) > 0 || error("Weights/tolerance must be positive")
    # The inverse in the original cost builder requires observable translations.
    Qt = translation_cost(A, B, ones(n))
    singular_values = svdvals(Qt[1:6, 1:6])
    singular_values[end] > 1e-8 * singular_values[1] ||
        error("Unobservable/ill-conditioned motion: rotate about multiple nonparallel axes")
    cost, Q = transformation_robot_world_cost(A, B, fill(kappa, n),
                                               fill(0.5 / translation_sigma_m^2, n))
    cost = Matrix(Symmetric((cost + cost') / 2))
    constraints, rhs = get_double_rotation_constraints(true, true, 3)
    # Solving the primal relaxation of the same QCQP exposes equality duals and
    # the lifted matrix, allowing independent feasibility and gap checks.
    scale = max(opnorm(cost), 1.0)
    model = Model(optimizer_with_attributes(COSMO.Optimizer,
        "eps_abs" => 1e-9, "eps_rel" => 1e-9, "max_iter" => 100000,
        "verbose" => false))
    @variable(model, Z[1:19, 1:19], PSD)
    eq = @constraint(model, [i=1:length(rhs)],
                     sum(constraints[i, j, k] * Z[j, k] for j=1:19, k=1:19) == rhs[i])
    @objective(model, Min, sum(cost[j, k] / scale * Z[j, k] for j=1:19, k=1:19))
    optimize!(model)
    status = termination_status(model)
    status in (MOI.OPTIMAL, MOI.ALMOST_OPTIMAL) || error("SDP failed: $status")
    has_values(model) && has_duals(model) || error("SDP did not return primal/dual solutions")
    lifted = Matrix(Symmetric(value.(Z)))
    eig = eigen(Symmetric(lifted))
    v = eig.vectors[:, end]
    abs(v[end]) > 1e-6 || error("Relaxation cannot be rounded; homogenizing coordinate is zero")
    v /= v[end]
    Rx = project_rotation(reshape(v[1:9], 3, 3))
    Ry = project_rotation(reshape(v[10:18], 3, 3))
    r = [vec(Rx); vec(Ry); 1.0]
    t = -(Q[1:6, 1:6] \ (Q[1:6, 7:end] * r))
    X = [Rx t[1:3]; 0 0 0 1]
    Y = [Ry t[4:6]; 0 0 0 1]
    q = [t; r]
    primal_cost = dot(q, Q * q)
    multipliers = dual.(eq) .* scale
    slack = cost - sum(multipliers[i] * constraints[i, :, :] for i=1:length(rhs))
    min_slack = eigmin(Symmetric(slack))
    dual_value = dot(multipliers, rhs)
    # Every feasible lifted rotation point has trace 3+3+1 = 7.
    # Thus d + 7*min(0,lambda_min(slack)) is a conservative numerical bound,
    # even when the returned dual slack has a slightly negative eigenvalue.
    lower_bound = dual_value + 7 * min(0.0, min_slack)
    gap = primal_cost - lower_bound
    gap_normalized = gap / max(1.0, abs(primal_cost), abs(lower_bound))
    feasibility = maximum(abs(sum(constraints[i, :, :] .* lifted) - rhs[i])
                          for i=1:length(rhs))
    rank_ratio = max(0.0, eig.values[end-1]) / max(eig.values[end], eps())
    certified = (status == MOI.OPTIMAL && -tolerance <= gap_normalized <= tolerance
                 && feasibility <= tolerance && eig.values[1] >= -tolerance
                 && rank_ratio <= tolerance)
    errors = zeros(n, 2)
    for i=1:n
        left = A[i, :, :] * X
        right = Y * B[i, :, :]
        errors[i, 1] = norm(left[1:3, 4] - right[1:3, 4])
        cosine = clamp((tr(left[1:3, 1:3]' * right[1:3, 1:3]) - 1) / 2, -1, 1)
        errors[i, 2] = rad2deg(acos(cosine))
    end
    report = Dict(
        "sample_count" => n, "solver" => "COSMO", "solver_status" => string(status),
        "known_scale" => true, "kappa" => kappa, "translation_sigma_m" => translation_sigma_m,
        "numerically_certified" => certified, "certificate_tolerance" => tolerance,
        "certificate_note" => "Floating-point evidence for the specified objective, not an exact proof or an accuracy guarantee",
        "primal_cost" => primal_cost, "dual_value" => dual_value,
        "conservative_lower_bound" => lower_bound, "gap" => gap,
        "normalized_gap" => gap_normalized, "dual_slack_min_eigenvalue" => min_slack,
        "sdp_constraint_max_error" => feasibility, "sdp_rank_ratio" => rank_ratio,
        "translation_condition_number" => singular_values[1] / singular_values[end],
        "translation_rms_m" => sqrt(sum(abs2, errors[:, 1]) / n),
        "rotation_rms_deg" => sqrt(sum(abs2, errors[:, 2]) / n))
    return X, Y, report, errors
end

function main(args)
    if length(args) < 3 || "--help" in args
        println("Usage: scripts/julia_rb5.sh scripts/calibrate_rb5.jl COMBINED_DIR OUTPUT_DIR eye_to_hand|eye_in_hand [kappa=125] [sigma_m=0.01]")
        return "--help" in args ? 0 : 1
    end
    folder, output, mount = args[1:3]
    mount in ("eye_to_hand", "eye_in_hand") || error("Unknown mount: $mount")
    convention_file = joinpath(folder, "convention.toml")
    if isfile(convention_file)
        convention = TOML.parsefile(convention_file)
        convention["mount"] == mount || error("Mount disagrees with the exported dataset")
        convention["units"] == "metres" || error("Dataset must use metres")
    end
    ispath(output) && error("Output already exists; choose a new results directory")
    A = read_poses(joinpath(folder, "tag_0_cam_0_A.csv"))
    B = read_poses(joinpath(folder, "tag_0_cam_0_B.csv"))
    kappa = length(args) >= 4 ? parse(Float64, args[4]) : 125.0
    sigma = length(args) >= 5 ? parse(Float64, args[5]) : 0.01
    X, Y, report, errors = solve_calibration(A, B; kappa=kappa, translation_sigma_m=sigma)
    report["mount"] = mount
    report["X"] = mount == "eye_to_hand" ? "T_tcp_board" : "T_tcp_camera"
    report["Y"] = mount == "eye_to_hand" ? "T_base_camera" : "T_base_board"
    report["transform_convention"] = "T_parent_child maps child coordinates to parent; translations in metres"
    mkpath(output)
    writedlm(joinpath(output, report["X"] * ".csv"), X, ',')
    writedlm(joinpath(output, report["Y"] * ".csv"), Y, ',')
    if mount == "eye_to_hand"
        writedlm(joinpath(output, "T_camera_base.csv"), inv(Y), ',')
    end
    open(joinpath(output, "residuals.csv"), "w") do io
        println(io, "translation_m,rotation_deg")
        writedlm(io, errors, ',')
    end
    open(joinpath(output, "report.toml"), "w") do io
        TOML.print(io, report)
    end
    println("Saved results: ", abspath(output))
    println("Numerically certified: ", report["numerically_certified"])
    println("Normalized gap: ", report["normalized_gap"])
    println("Residual RMS: ", report["translation_rms_m"] * 1000, " mm, ",
            report["rotation_rms_deg"], " degrees")
    return 0
end

if abspath(PROGRAM_FILE) == @__FILE__
    exit(main(ARGS))
end
