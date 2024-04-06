import matplotlib.pyplot as plt
from matplotlib.backend_bases import PickEvent


class CurveDrawer:
    def _init_(self, image_path):
        self.image_path = image_path
        self.coordinates = []
        self.curve_line, = plt.plot([], [], 'ro', picker=5)  # Enable picking for the data points

    def on_pick(self, event):
        if isinstance(event, PickEvent) and event.mouseevent.name == 'button_press_event':
            x, y = event.mouseevent.xdata, event.mouseevent.ydata
            self.coordinates.append((x, y))
            self.update_curve()

    def update_curve(self):
        if self.coordinates:
            x, y = zip(*self.coordinates)
            self.curve_line.set_xdata(x)
            self.curve_line.set_ydata(y)
            plt.draw()

    def draw_curve(self):
        # Load the image
        img = plt.imread(self.image_path)

        # Display the image
        fig, ax = plt.subplots()
        ax.imshow(img)

        # Connect the mouse click event to the on_pick function
        fig.canvas.mpl_connect('pick_event', self.on_pick)

        # Show the plot
        plt.show()

    def save_coordinates(self, output_file):
        with open(output_file, 'w') as file:
            for x, y in self.coordinates:
                file.write(f'{x},{y}\n')


if __name_ == "_main_":
    image_path = r"C:\Users\Umid\Desktop\SDP\path-of-life.jpg"
    output_file = r"C:\Users\Umid\Desktop\SDP\path.txt"

    curve_drawer = CurveDrawer(image_path)
    curve_drawer.draw_curve()
    curve_drawer.save_coordinates(output_file)
