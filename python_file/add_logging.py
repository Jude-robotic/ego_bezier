import re
file_path = 'src/planner/bspline_opt/src/bspline_optimizer.cpp'
with open(file_path, 'r') as f:
    content = f.read()

log_code = """
    if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP || result == lbfgs::LBFGS_ALREADY_MINIMIZED)
    {
        // Success
    }
    else
    {
        std::cout << "[BsplineOptimizer] Optimization failed: " << lbfgs::lbfgs_strerror(result) << " (" << result << ")" << std::endl;
    }
"""

# Find all occurrences of lbfgs_optimize call ending with ;
# and append log_code
pattern = r"(int result = lbfgs::lbfgs_optimize\(.*?\);)"
content = re.sub(pattern, r"\1" + log_code, content, flags=re.DOTALL)

with open(file_path, 'w') as f:
    f.write(content)
