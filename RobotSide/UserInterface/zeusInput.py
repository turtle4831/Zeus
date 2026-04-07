from pydualsense import pydualsense

cancelRequest = False
ds = pydualsense()

def start():
    ds.init()
    
def getControls():
    return ds.state
    
def end():
    ds.close()

def clamp(value, min_val, max_val):
    """
    Clamps a value within a specified range [min_val, max_val].
    """
    return max(min_val, min(value, max_val))

def rescale_value(value, original_min, original_max, target_min, target_max):
    """
    Rescales a single value from an original range to a target range.
    """
    if original_max - original_min == 0:
        return target_min + (target_max - target_min) / 2 # Handle division by zero for constant data
        
    scaled_to_01 = (value - original_min) / (original_max - original_min)
    scaled_to_target = target_min + scaled_to_01 * (target_max - target_min)
    
    return scaled_to_target

ds.init()

yMovement = rescale_value(getControls().LY, -128, 127, -1, 1)
xMovement = rescale_value(getControls().LX, -128, 127, -1, 1)
Turn = rescale_value(getControls().RX, -128, 127, -1, 1)