import abc
import math
from PIL import Image, ImageFont, ImageDraw

# yapf: disable
TARGET_TYPES = [
    "Circle",
    "SemiCircle",
    "QuarterCircle",
    "Triangle",
    "Square",
    "Rectangle",
    "Trapezoid",
    "Pentagon",
    "Hexagon",
    "Heptagon",
    "Octagon",
    "Star",
    "Cross"
]
#yapf: enable


class TargetGenerator(abc.ABC):
    def __init__(self, font_path, font_ratio=0.35, font_face=0):
        """Generate targets.

        Arguments:
        font_path -- path to OpenType/TrueType font
        font_ratio -- font_height/target_size
        font_face -- index of the font face to load
        """
        self.font_path = font_path
        self.font_ratio = font_ratio
        self.font_face = font_face
        self.font_size = 0
        self.font = ImageFont.truetype(
            font=font_path, size=self.font_size, index=font_face)

    @abc.abstractmethod
    def _draw_shape(self, context, size, color):
        """Draw the desired shape."""

    def _draw_letter(self, size, letter, color):
        # Resize font to correct ratio
        while (self.font.getsize(letter)[1] / size) > self.font_ratio:
            self.font_size -= 1
            self.font = ImageFont.truetype(
                font=self.font_path, size=self.font_size, index=self.font_face)

        while (self.font.getsize(letter)[1] / size) < self.font_ratio:
            self.font_size += 1
            self.font = ImageFont.truetype(
                font=self.font_path, size=self.font_size, index=self.font_face)

        # Draw the letter
        font = self.font
        image = Image.new('RGBA', (size, size), (255, 255, 255, 0))
        context = ImageDraw.Draw(image)
        font_width, font_height = font.getsize(letter)
        text_corner = (int((size / 2) - (font_width / 2)),
                       int((size / 2) - (font_height / 2)))

        context.text(text_corner, letter, font=font, fill=color + (255, ))
        return image

    def draw_target(self, size, shape_color, letter, letter_color):
        base_shape = Image.new('RGBA', (size, size), (255, 255, 255, 0))
        context = ImageDraw.Draw(base_shape)
        self._draw_shape(context, size, shape_color)

        return Image.alpha_composite(
            base_shape, self._draw_letter(size, letter, letter_color))


class Circle(TargetGenerator):
    def _draw_shape(self, context, size, color):
        context.ellipse([(0, 0), (size, size)], fill=color + (255, ))


class SemiCircle(TargetGenerator):
    def _draw_shape(self, context, size, color):
        context.pieslice(
            [(0, int(size / 4)), (size, int(size * 5 / 4))],
            180,
            360,
            fill=color + (255, ))


class QuarterCircle(TargetGenerator):
    def _draw_shape(self, context, size, color):
        context.pieslice(
            [(-size, 0), (size, size * 2)], 270, 360, fill=color + (255, ))


class Square(TargetGenerator):
    def _draw_shape(self, context, size, color):
        context.rectangle([(0, 0), (size, size)], fill=color + (255, ))


class Rectangle(TargetGenerator):
    def draw_target(self, size, shape_color, letter, letter_color, dim_ratio):
        """Draw a rectangle target.
        
        Arguments:
        size -- width and height (one integer) of the target
        shape_color -- color of the shape
        letter -- letter to draw
        letter_color -- color of the letter
        dim_ratio -- ratio of the height / width of the rectangle
        """

        self.dim_ratio = dim_ratio
        return super().draw_target(size, shape_color, letter, letter_color)

    def _draw_shape(self, context, size, color):
        if self.dim_ratio < 1:
            top_left = (0, (size - size * self.dim_ratio) / 2)
            bot_right = (size, top_left[1] + size * self.dim_ratio)
            context.rectangle([top_left, bot_right], fill=color + (255, ))
        else:
            top_left = ((size - size / self.dim_ratio) / 2, 0)
            bot_right = (top_left[0] + size / self.dim_ratio, size)
            context.rectangle([top_left, bot_right], fill=color + (255, ))


# yapf: disable
class Trapezoid(TargetGenerator):
    def _draw_shape(self, context, size, color):
        context.polygon([(int(size / 4), int(size / 4)),
                         (int(size * 3 / 4), int(size / 4)),
                         (size, int(size * 3 / 4)),
                         (0, int(size * 3 / 4))], fill=color + (255, ))
# yapf: enable


class Triangle(TargetGenerator):
    def _draw_shape(self, context, size, color):
        context.polygon(
            [(int(size / 2), 0), (size, size), (0, size)],
            fill=color + (255, ))


# yapf: disable
class Cross(TargetGenerator):
    def _draw_shape(self, context, size, color):
        context.polygon(
            [(int(size / 4), 0),
             (int(size * 3 / 4), 0),
             (int(size * 3 / 4), int(size / 4)),
             (size, int(size / 4)),
             (size, int(size * 3 / 4)),
             (int(size * 3 / 4), int(size * 3 / 4)),
             (int(size * 3 / 4), size),
             (int(size / 4), size),
             (int(size / 4), int(size * 3 / 4)),
             (0, int(size * 3 / 4)),
             (0, int(size / 4)),
             (int(size / 4), int(size / 4))],
            fill=color + (255, ))
# yapf: enable


class Polygon(TargetGenerator):
    def _draw_polygon(self, context, size, color, n_sides):

        # the first point is always on the top edge
        verticies = [(int(size / 2), 0)]

        r = size / 2
        d_angle = math.pi * 2 / n_sides
        angle = math.pi / 2 + d_angle

        for i in range(n_sides - 1):
            x = math.cos(angle) * r + r
            y = math.sin(angle) * -r + r
            verticies += [(int(x), int(y))]
            angle += d_angle

        #print('Creating polygon ' + str(n_sides) + ': ' + str(verticies))
        context.polygon(verticies, fill=color + (255, ))


class Pentagon(Polygon):
    def _draw_shape(self, context, size, color):
        self._draw_polygon(context, size, color, 5)


class Hexagon(Polygon):
    def _draw_shape(self, context, size, color):
        self._draw_polygon(context, size, color, 6)


class Heptagon(Polygon):
    def _draw_shape(self, context, size, color):
        self._draw_polygon(context, size, color, 7)


class Octagon(Polygon):
    def _draw_shape(self, context, size, color):
        self._draw_polygon(context, size, color, 8)

class Star(TargetGenerator):
    def _draw_shape(self, context, size, color):

        # the first point is always on the top edge
        verticies = [(int(size / 2), 0)]

        n_points = 5

        out_r = size / 2
        in_r = out_r * 1 / 2
        d_angle = math.pi * 2 / n_points
        out_angle = math.pi / 2 + d_angle
        in_angle = math.pi / 2 + d_angle / 2

        for i in range(n_points - 1):
            in_x = math.cos(in_angle) * in_r + out_r
            in_y = math.sin(in_angle) * -in_r + out_r
            verticies += [(int(in_x), int(in_y))]
            in_angle += d_angle

            out_x = math.cos(out_angle) * out_r + out_r
            out_y = math.sin(out_angle) * -out_r + out_r
            verticies += [(int(out_x), int(out_y))]
            out_angle += d_angle

        # final vertex
        in_x = math.cos(in_angle) * in_r + out_r
        in_y = math.sin(in_angle) * -in_r + out_r
        verticies += [(int(in_x), int(in_y))]

        context.polygon(verticies, fill=color + (255, ))

