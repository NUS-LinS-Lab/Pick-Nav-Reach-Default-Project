import numpy as np

def wrap_to_pi(x):
    return (x + np.pi) % (2 * np.pi) - np.pi

def closest_joint_values(target_joint_values, current_joint_values, wrap_mask):
    """
    Args:
        target_joint_values: np.ndarray, shape (N,)
        current_joint_values: np.ndarray, shape (N,)
        wrap_mask: np.ndarray[bool or int], shape (N,)  # 1 表示需要 wrap, 0 表示直接保留

    Returns:
        output_joint_values: np.ndarray, shape (N,)
    """
    target_joint_values = np.asarray(target_joint_values, dtype=float)
    current_joint_values = np.asarray(current_joint_values, dtype=float)
    wrap_mask = np.asarray(wrap_mask, dtype=bool)

    output_joint_values = target_joint_values.copy()

    diffs = wrap_to_pi(target_joint_values[wrap_mask] - current_joint_values[wrap_mask])
    output_joint_values[wrap_mask] = current_joint_values[wrap_mask] + diffs

    return output_joint_values