from PIL import Image
import numpy as np

def image2array(im):
    if im.mode not in ("L", "F"):
        raise ValueError, "can only convert single-layer images"
    if im.mode == "L":
        a = np.fromstring(im.tostring(), dtype=np.uint8)
    else:
        a = np.fromstring(im.tostring(), dtype=np.float32)
    a.shape = im.size[1], im.size[0]
    return a

def array2image(a):
    if a.typecode() == np.uint8:
        mode = "L"
    elif a.typecode() == np.float32:
        mode = "F"
    else:
        raise ValueError, "unsupported image mode %s"%(a.typecode())
    return Image.fromstring(mode, (a.shape[1], a.shape[0]), a.tostring())


