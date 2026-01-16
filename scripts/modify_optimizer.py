import os

file_path = 'src/planner/bspline_opt/src/bspline_optimizer.cpp'

with open(file_path, 'r') as f:
    content = f.read()

# Replacement for rebound_optimize
old_rebound = """    int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
    // Copy back
    cps_.points = Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols());
    
    return (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP);"""

new_rebound = """    int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
    
    if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP || result == lbfgs::LBFGS_ALREADY_MINIMIZED)
    {
        // Success
    }
    else
    {
        std::cout << "[BsplineOptimizer] Rebound optimization failed: " << lbfgs::lbfgs_strerror(result) << " (" << result << ")" << std::endl;
    }

    // Copy back
    cps_.points = Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols());
    
    return (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP || result == lbfgs::LBFGS_ALREADY_MINIMIZED);"""

# Replacement for refine_optimize
old_refine = """    int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
    cps_.points = Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols());

    return (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP);"""

new_refine = """    int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
    
    if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP || result == lbfgs::LBFGS_ALREADY_MINIMIZED)
    {
        // Success
    }
    else
    {
        std::cout << "[BsplineOptimizer] Refine optimization failed: " << lbfgs::lbfgs_strerror(result) << " (" << result << ")" << std::endl;
    }

    cps_.points = Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols());

    return (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP || result == lbfgs::LBFGS_ALREADY_MINIMIZED);"""

if old_rebound in content:
    content = content.replace(old_rebound, new_rebound)
    print("Replaced rebound_optimize logic.")
else:
    print("Could not find rebound_optimize logic.")
    # Debug again if needed
    start_idx = content.find("int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, BsplineOptimizer::costFunctionRebound")
    if start_idx != -1:
        print("Found start of rebound logic at", start_idx)
        print("Next 200 chars:", content[start_idx:start_idx+200])

if old_refine in content:
    content = content.replace(old_refine, new_refine)
    print("Replaced refine_optimize logic.")
else:
    print("Could not find refine_optimize logic.")

with open(file_path, 'w') as f:
    f.write(content)

