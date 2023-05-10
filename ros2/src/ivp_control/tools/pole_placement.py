from math import cos, sin
from numpy import matrix, array, sign
import control


def generate_state_space():
    g = 9.81
    m_p = 0.071
    m_c = 0.288
    b_c = 0.095
    b_p = 0.00112297
    l_p = (0.685 - 0.246)
    i_p = 0.00466
    num = (m_c + m_p)*i_p + m_c*m_p*l_p**2
    a_22 = (-b_c * (m_p * l_p ** 2 + i_p)) / num
    a_23 = (l_p ** 2 * g * m_p ** 2) / num
    a_24 = (-b_p * l_p * m_p) / num
    a_42 = (-b_c * m_p * l_p) / num
    a_43 = (m_p * g * l_p * (m_c + m_p)) / num
    a_44 = (-b_p * (m_c + m_p)) / num
    A = matrix([
        [0, 1, 0, 0],
        [0, a_22, a_23, a_24],
        [0, 0, 0, 1],
        [0, a_42, a_43, a_44]
    ])
    b_2 = (m_p * l_p ** 2 + i_p) / num
    b_4 = m_p * l_p / num
    B = matrix([
        [0],
        [b_2],
        [0],
        [b_4]
    ])
    Q = matrix([
        [10, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 100, 0],
        [0, 0, 0, 1]
    ])
    R = 3
    # R = 0.01

    K, S, E = control.lqr(A, B, Q, R)
    return A, B, K


if __name__ == "__main__":
    A, B, K = generate_state_space()
    print(A)
    print(B)
    print(K)
