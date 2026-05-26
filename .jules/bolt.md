## 2024-05-27 - Python Power Operator Performance
**Learning:** In Python, the exponentiation operator (`** 2`) is significantly slower than direct multiplication (`x * x`) due to the overhead of the generalized power function. This can be a noticeable bottleneck in math-heavy operations like those in kinematics.
**Action:** Always prefer direct multiplication (`x * x`) over exponentiation (`x ** 2`) in performance-critical code sections, especially for small, fixed exponents.
