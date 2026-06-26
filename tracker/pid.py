"""
tracker/pid.py
==============
Generic PID controller with:
  • Anti-windup (integral clamping)
  • Derivative filtering (low-pass on D term to reduce noise)
  • Output clamping
  • Reset on target loss
"""


class PIDController:
    """
    Discrete PID controller.

    Usage:
        pid = PIDController(kp=0.1, ki=0.01, kd=0.05, output_limit=1.5)
        cmd = pid.update(error, dt)
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float = float("inf"),
        integral_limit: float = 50.0,    # Anti-windup clamp (pixel units)
        derivative_filter: float = 0.7,  # Low-pass coefficient for D term
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit   = output_limit
        self.integral_limit = integral_limit
        self.deriv_alpha    = derivative_filter

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_deriv = 0.0

    def update(self, error: float, dt: float) -> float:
        """
        Compute PID output for one timestep.

        Args:
            error: Current error (e.g., pixel offset from center)
            dt:    Time since last call (seconds)

        Returns:
            Control output (clamped to ±output_limit)
        """
        if dt <= 0:
            return 0.0

        # ── Proportional ──────────────────────────────────────────────────────
        p = self.kp * error

        # ── Integral with anti-windup clamp ───────────────────────────────────
        self._integral += error * dt
        self._integral  = max(-self.integral_limit,
                              min(self.integral_limit, self._integral))
        i = self.ki * self._integral

        # ── Derivative with low-pass filter ───────────────────────────────────
        raw_deriv  = (error - self._prev_error) / dt
        filtered_d = (self.deriv_alpha * self._prev_deriv +
                      (1.0 - self.deriv_alpha) * raw_deriv)
        self._prev_deriv = filtered_d
        self._prev_error = error
        d = self.kd * filtered_d

        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self) -> None:
        """Call when tracking is lost to avoid integral windup on re-acquire."""
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_deriv = 0.0
