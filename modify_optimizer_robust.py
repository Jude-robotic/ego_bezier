import os
import re

file_path = 'src/planner/bspline_opt/src/bspline_optimizer.cpp'

with open(file_path, 'r') as f:
    content = f.read()

# Regex for rebound_optimize
# We look for the call to lbfgs_optimize, followed by // Copy back, followed by return
pattern_rebound = r"(int result = lbfgs::lbfgs_optimize\(variable_num_, q\.data\(\), &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params\);)\s+// Copy back\s+cps_\.points = Eigen::Map<Eigen::MatrixXd>\(q\.data\(\), 3, cps_\.points\.cols\(\)\);\s+return \(result == lbfgs::LBFGS_CONVERGENCE \|\| result == lbfgs::LBFGS_STOP\);"

replacement_rebound = r"""int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);

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

# Regex for refine_optimize
pattern_refine = r"(int result = lbfgs::lbfgs_optimize\(variable_num_, q\.data\(\), &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params\);)\s+cps_\.points = Eigen::Map<Eigen::MatrixXd>\(q\.data\(\), 3, cps_\.points\.cols\(\)\);\s+return \(result == lbfgs::LBFGS_CONVERGENCE \|\| result == lbfgs::LBFGS_STOP\);"

replacement_refine = r"""int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);

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

# Perform replacement
new_content = re.sub(pattern_rebound, replacement_rebound, content, flags=re.DOTALL)
if new_content != content:
    print("Replaced rebound_optimize logic.")
    content = new_content
else:
    print("Could not find rebound_optimize logic with regex.")

new_content = re.sub(pattern_refine, replacement_refine, content, flags=re.DOTALL)
if new_content != content:
    print("Replaced refine_optimize logic.")
    content = new_content
else:
    print("Could not find refine_optimize logic with regex.")

with open(file_path, 'w') as f:
    f.write(content)

