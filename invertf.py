# invert_tf.py

def quat_inv(q):
    # Inverse of a quaternion [x, y, z, w] is [-x, -y, -z, w]
    return [-q[0], -q[1], -q[2], q[3]]

def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1[3], q1[0], q1[1], q1[2]
    w2, x2, y2, z2 = q2[3], q2[0], q2[1], q2[2]
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]

def rotate_vec(q, v):
    q_vec = [v[0], v[1], v[2], 0.0]
    q_inv_val = quat_inv(q)
    temp = quat_mult(q, q_vec)
    res = quat_mult(temp, q_inv_val)
    return [res[0], res[1], res[2]]

# Your original MATLAB values
t = [0.050004, -0.204872, -0.065184]
q = [ -0.330842, 0.317747, -0.634132, 0.622461]

# Calculate the inverse
q_inv = quat_inv(q)
t_rotated = rotate_vec(q_inv, t)
t_inv = [-t_rotated[0], -t_rotated[1], -t_rotated[2]]

print(f"--- NEW INVERTED COMMAND ARGUMENTS ---")
print(f"Translation: {t_inv[0]:.6f} {t_inv[1]:.6f} {t_inv[2]:.6f}")
print(f"Quaternion:  {q_inv[0]:.6f} {q_inv[1]:.6f} {q_inv[2]:.6f} {q_inv[3]:.6f}")
