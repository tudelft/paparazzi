from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import sys
import warnings

if sys.version_info[0] < 3:
    warnings.warn("This script should run using Python 3, which is currently not the case. The plot might not generate correctly.")

#path = '../ROC_dataset/'


def my_obstacle_filter(im, param):
    """ An example of a filter that can be used to generate ROC curves.
    This filter is a simple color filter around orange. Replace this with your own filter.

    :param im: image to be filtered
    :param param: filter parameter that will be varied between 0.0 and 1.0
    :return: filtered image where detected objects have a value of [255,255,255]
    """
    # Set up the filter based on the input parameter
    filter_width = np.array([0, 0, 0]) + param*255
    color_green = np.array([137,255,71])

    w, h = im.size

    # Load pixel data
    im_pixels = np.asarray(im.getdata(), dtype=int)

    # Create mask for detected obstacles
    mask = np.all(im_pixels > color_green - filter_width, axis=1) * np.all(im_pixels < color_green + filter_width,
                                                                            axis=1)
    # Create image where obstacles are white
    filtered_im = Image.new('RGB', (w, h), color=(0, 0, 0))
    filtered_im_pixels = np.asarray(filtered_im.getdata())
    filtered_im_pixels[mask] = [255, 255, 255]
    filtered_im.putdata([tuple(pix) for pix in filtered_im_pixels])

    return filtered_im


def generate_ROC_plot():
    """ Generates a simple ROC plot"""
    plot_data = []
    n_images = 5    # Number of images in folder
    for param in np.linspace(0.0, 1.0, 10):
        # Initialize totals
        true_positives = 0
        false_positives = 0
        ground_truth_positives = 0
        ground_truth_negatives = 0

        for i in range(1, n_images + 1):
            # Set image paths
            original_path = 'test_open/original_' + str(i) + '.jpg'
            ground_truth_path = 'test_open/mask_' + str(i) + '.jpg'

            # Analyze ground truth image
            ground_truth_im = Image.open(ground_truth_path, 'r')
            ground_truth_pixels = np.asarray(ground_truth_im.getdata())
            ground_truth_obstacles = np.all(ground_truth_pixels == [255, 255, 255], axis=1)

            # Analyze original image
            im = Image.open(original_path, 'r')
            filtered_im = my_obstacle_filter(im, param)
            filtered_im_pixels = np.asarray(filtered_im.getdata())
            filtered_im_obstacles = np.all(filtered_im_pixels == [255, 255, 255], axis=1)

            # Update totals of positives/negatives
            true_positives += np.sum((filtered_im_obstacles == True) & (ground_truth_obstacles == True))
            false_positives += np.sum((filtered_im_obstacles == True) & (ground_truth_obstacles == False))

            ground_truth_positives += np.sum((ground_truth_obstacles == True))
            ground_truth_negatives += np.sum((ground_truth_obstacles == False))

        # Calculate rates
        false_positive_rate = false_positives / ground_truth_negatives
        true_positive_rate = true_positives / ground_truth_positives

        # Add datapoint to plot_data
        plot_data.append((false_positive_rate, true_positive_rate))
        print(false_positive_rate, true_positive_rate)

    # Create x and y data from plot_data
    x = [item[0] for item in plot_data]
    y = [item[1] for item in plot_data]

    # Plot
    plt.plot(x, y,linewidth=8.0)
    #plt.title("ROC curve",fontsize=20, fontweight='bold')
    plt.xlabel("False Positive Rate",fontsize=20, fontweight='bold')
    plt.ylabel("True Positive Rate",fontsize=20, fontweight='bold')
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.grid()
    plt.savefig("ROC_final.svg")
    plt.show()


# Main script
generate_ROC_plot()
