class PolynomialInterpolator:
    """
    Clase base para los interpoladores polinómicos.
    Trabaja con listas de 6 elementos para las 6 articulaciones del UR3.
    """
    def __init__(self, q_start, q_end, duration):
        self.q_start = q_start
        self.q_end = q_end
        self.duration = float(duration)
        self.num_joints = len(q_start)

class CubicInterpolator(PolynomialInterpolator):
    def evaluate(self, current_time):
        # Saturación por seguridad: si el tiempo está fuera del rango, devolvemos los extremos
        if current_time <= 0.0:
            return self.q_start, [0.0] * self.num_joints
        if current_time >= self.duration:
            return self.q_end, [0.0] * self.num_joints

        # Tiempo normalizado tau (de 0.0 a 1.0)
        tau = current_time / self.duration

        # Ecuaciones del polinomio cúbico (Perfil de velocidad parabólico)
        c_pos = 3 * (tau**2) - 2 * (tau**3)
        c_vel = (6 * tau - 6 * (tau**2)) / self.duration

        # Calculamos la posición y velocidad para cada una de las 6 articulaciones
        q_ref = [self.q_start[i] + (self.q_end[i] - self.q_start[i]) * c_pos for i in range(self.num_joints)]
        dq_ref = [(self.q_end[i] - self.q_start[i]) * c_vel for i in range(self.num_joints)]

        return q_ref, dq_ref

class QuinticInterpolator(PolynomialInterpolator):
    def evaluate(self, current_time):
        if current_time <= 0.0:
            return self.q_start, [0.0] * self.num_joints
        if current_time >= self.duration:
            return self.q_end, [0.0] * self.num_joints

        tau = current_time / self.duration

        # Ecuaciones del polinomio quíntico (Perfil de aceleración continuo)
        c_pos = 10 * (tau**3) - 15 * (tau**4) + 6 * (tau**5)
        c_vel = (30 * (tau**2) - 60 * (tau**3) + 30 * (tau**4)) / self.duration

        q_ref = [self.q_start[i] + (self.q_end[i] - self.q_start[i]) * c_pos for i in range(self.num_joints)]
        dq_ref = [(self.q_end[i] - self.q_start[i]) * c_vel for i in range(self.num_joints)]

        return q_ref, dq_ref