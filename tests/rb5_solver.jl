using Test, Random
include(joinpath(@__DIR__, "..", "scripts", "calibrate_rb5.jl"))

@testset "Original RWHE objective, SDP, and numerical certificate" begin
    Random.seed!(850)
    pose() = [Matrix(rand(RotMatrix{3})) randn(3) .* 0.2; 0 0 0 1]
    X, Y = pose(), pose()
    A, B = zeros(30, 4, 4), zeros(30, 4, 4)
    for i=1:30
        A[i, :, :] = pose()
        B[i, :, :] = inv(Y) * A[i, :, :] * X
    end
    # Check the authors' constraints on a known feasible rotation pair.
    C, b = get_double_rotation_constraints()
    r = [vec(X[1:3, 1:3]); vec(Y[1:3, 1:3]); 1]
    @test maximum(abs(dot(r, C[i, :, :] * r) - b[i]) for i=1:length(b)) < 1e-12
    Xhat, Yhat, report, errors = solve_calibration(A, B)
    @test norm(Xhat - X) < 1e-4
    @test norm(Yhat - Y) < 1e-4
    @test report["numerically_certified"]
    @test maximum(errors[:, 1]) < 1e-5
    @test report["conservative_lower_bound"] <= report["primal_cost"] + 1e-7
    # Noise must produce nonzero fit residuals; the lower bound still applies.
    Bnoisy = copy(B)
    Bnoisy[:, 1:3, 4] .+= 0.001 .* randn(30, 3)
    _, _, noisy, _ = solve_calibration(A, Bnoisy)
    @test noisy["translation_rms_m"] > 1e-4
    @test noisy["conservative_lower_bound"] <= noisy["primal_cost"] + 1e-6
    degenerate = repeat(reshape(Matrix{Float64}(I, 4, 4), 1, 4, 4), 8, 1, 1)
    @test_throws ErrorException solve_calibration(degenerate, degenerate)
    @test_throws ErrorException solve_calibration(A[1:7, :, :], B[1:7, :, :])
    @test_throws ErrorException solve_calibration(A, B[1:29, :, :])
end
