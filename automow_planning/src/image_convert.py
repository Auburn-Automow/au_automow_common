import sys
from PIL import Image

def to_jpg(img):
    """
    A utility for converting data structures to jpegs.
    """
    pass


def save_as_jpg(file_name):
    "Saves an image as a jpeg."
    im = Image.open(file_name)
    im.save(file_name+".jpg", "JPEG")


def main():
    """Convert a given image to a jpg, useful for converting png, gif, etc."""
    if len(sys.argv) < 2:
        print "Pass the path to an image that needs to be converted."
        sys.exit(0)
    save_as_jpg(sys.argv[1])


if __name__ == '__main__':
    main()
