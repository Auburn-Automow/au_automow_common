import shapely.geometry as geo
import numpy as np

resolution_ = 100 # 100 == centimeters

def image2array(im):
    if im.mode not in ("L", "F"):
        raise ValueError, "can only convert single-layer images"
    if im.mode == "L":
        a = np.fromstring(im.tostring(), dtype=np.uint8)
    else:
        a = np.fromstring(im.tostring(), dtype=np.float32)
    a.shape = im.size[1], im.size[0]
    return a

def update_coverage_map(self, cutter, state, resolution=100):
    """
    Takes the cutter shape and state, then updates the coverage map.
    """
    if state: # If this cutter is on
        # Get the shape in the image frame
        cutter_coords = []
        from math import ceil
        resolution = float(resolution)
        offset = (cutter.bounds[0], cutter.bounds[1])
        size = (cutter.bounds[2]-cutter.bounds[0])
        for coord in list(cutter.exterior.coords):
            # Convert from meters to image resolution
            new_coord = [coord[0]-offset[0], coord[1]-offset[1]]
            new_coord[0] *= resolution
            new_coord[1] *= resolution
            cutter_coords.append(tuple(new_coord))
        # Raster the polygon into an image using PIL
        import Image, ImageDraw
        dim = int(ceil(size*resolution))
        im = Image.new("L", (dim, dim), 0)
        draw = ImageDraw.Draw(im)
        draw.polygon(cutter_coords, fill=255, outline=255)
        del draw
        # Get the "cut" pixels
        cut_pixels = []
        for i, row in enumerate(image2array(im)):
            for j, element in enumerate(row):
                if element != 0:
                    pixel_pair = [i/resolution, j/resolution, 0]
                    pixel_pair[0] += offset[0]
                    pixel_pair[1] += offset[1]
                    cut_pixels.append(pixel_pair)
        # Now put them in GridCells messages

def main():
    cutter = geo.Point([5,5]).buffer(0.3)
    update_coverage_map(None, cutter, True, resolution_)



if __name__ == '__main__':
    main()
