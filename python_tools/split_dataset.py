# split_dataset.py

# splits image dataset into train, val and test
# via copy-paste

# 2024 FEB 23
# Wonhee Lee

import shutil
import glob, os
import random

# user setting ////////////////////////////////////////////////////////////////
# set directory where images and their annotation files are stored
image_directory_path = "/home/tershire/Pictures/doors"
image_file_format = ".png"

# set percentages of images to be used for validation and test
percentage_val = 20  # [%]
percentage_test = 10  # [%]

# sanity check ////////////////////////////////////////////////////////////////
assert percentage_val + percentage_test < 100


# function ////////////////////////////////////////////////////////////////////
def pop_random_image_path(image_paths):
    image_path = image_paths.pop(random.randint(0, len(image_paths) - 1))
    return image_path


# split ///////////////////////////////////////////////////////////////////////
# create sub directories
os.mkdir(image_directory_path + "/train")
os.mkdir(image_directory_path + "/val")
os.mkdir(image_directory_path + "/test")

# load image paths
image_paths = list(glob.iglob(os.path.join(image_directory_path + "/", "*" + image_file_format)))

# calculate number of images
num_images = len(image_paths)
num_val_images = round(num_images*(percentage_val/100))
num_test_images = round(num_images*(percentage_test/100))
num_train_images = num_images - (num_val_images + num_test_images)

# copy validation images
val_image_paths = []
for i in range(0, num_val_images):
    val_image_paths.append(pop_random_image_path(image_paths))
val_image_paths.sort()
for image_path in val_image_paths:
    path_and_name = os.path.split(image_path)
    shutil.copyfile(image_path, os.path.join(path_and_name[0] + "/", "val/" + path_and_name[-1]))

    # annotation file
    text_name = path_and_name[-1].split(".")[0] + ".txt"
    text_path = os.path.join(path_and_name[0] + "/", text_name)
    shutil.copyfile(text_path, os.path.join(path_and_name[0] + "/", "val/" + text_name))

# copy test images
test_image_paths = []
for i in range(0, num_test_images):
    test_image_paths.append(pop_random_image_path(image_paths))
test_image_paths.sort()
for image_path in test_image_paths:
    path_and_name = os.path.split(image_path)
    shutil.copyfile(image_path, os.path.join(path_and_name[0] + "/", "test/" + path_and_name[-1]))

    # annotation file
    text_name = path_and_name[-1].split(".")[0] + ".txt"
    text_path = os.path.join(path_and_name[0] + "/", text_name)
    shutil.copyfile(text_path, os.path.join(path_and_name[0] + "/", "test/" + text_name))

# copy training images
train_image_paths = image_paths
for image_path in train_image_paths:
    path_and_name = os.path.split(image_path)
    shutil.copyfile(image_path, os.path.join(path_and_name[0] + "/", "train/" + path_and_name[-1]))

    # annotation file
    text_name = path_and_name[-1].split(".")[0] + ".txt"
    text_path = os.path.join(path_and_name[0] + "/", text_name)
    shutil.copyfile(text_path, os.path.join(path_and_name[0] + "/", "train/" + text_name))

# output //////////////////////////////////////////////////////////////////////
print("split\n",
      "\t-", len(train_image_paths), "images to train\n",
      "\t-", len(val_image_paths), "images to val\n",
      "\t-", len(test_image_paths), "images to test\n")
