def angle_diff(src: float, target: float) -> float:
    """
    Gets the (signed) distance between two angles in degrees
    :param src:
    :param target:
    :return:
    """
    return (src - target + 180) % 360 - 180
