import abc
from PIL import Image, ImageFont, ImageDraw

# yapf: disable
TARGET_TYPES = [
    "Circle",
    "HalfCircle",
    "QuarterCircle",
    "Rectangle",
    "Trapezoid",
    "Triangle",
    "Cross",
    "Pentagon",
    "Hexagon",
    "Heptagon",
    "Octagon",
    "Star"
]
#yapf: enable


class TargetGenerator(abc.ABC):
    def __init__(self, font_path, font_size=12, font_face=0):
        """Generate targets.

        Arguments:
        font_path -- path to OpenType/TrueType font
        font_size -- point size of the font
        font_face -- index of the font face to load
        """
        self.font = ImageFont.truetype(
            font=font_path, size=font_size, index=font_face)

    @abc.abstractmethod
    def _draw_shape(self, context, size, color):
        """Draw the desired shape."""

    def _draw_letter(self, size, letter, color):
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


class HalfCircle(TargetGenerator):
    def _draw_shape(self, context, size, color):
        context.pieslice(
            [(0, int(size / 4)), (size, int(size * 5 / 4))],
            180,
            360,
            fill=color + (255, ))


class QuarterCircle(TargetGenerator):
    def _draw_shape(self, context, size, color):
        context.pieslice([(-size, 0), (size, size * 2)], 270, 360, fill=color + (255, ))


class Rectangle(TargetGenerator):
    def _draw_shape(self, context, size, color):
        context.rectangle([(0, 0), (size, size)], fill=color + (255, ))


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

