#encoding:utf-8
import csv

samples = []
with open('driving_log.csv') as f:
    reader = csv.reader(f)
    for line in reader:
        samples.append(line)


from sklearn.model_selection import train_test_split
train_samplses, validation_samples = train_test_split(samples, test_size=0.2)

import numpy as np
import cv2
import sklearn

# Extract features from the data set.
def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1:
        sklearn.utils.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset : offset+batch_size]

            images = []
            angles = []

            correction = 0.35
            for batch_sample in batch_samples:               
                center_img = cv2.imread(batch_sample[0])
                images.append(center_img)
                center_angle = float(line[3])
                angles.append(center_angle)            

                left_img = cv2.imread(batch_sample[1])
                images.append(left_img)
                left_angle = float(line[3]) + correction
                angles.append(left_angle) 

                right_img = cv2.imread(batch_sample[2])
                images.append(right_img)
                right_angle = float(line[3]) - correction
                angles.append(right_angle)                                 

            augmented_images, augmented_angles = [], []
            for image, angle in zip(images, angles):
                augmented_images.append(image)
                augmented_angles.append(angle)
                augmented_images.append(cv2.flip(image, 1))
                augmented_angles.append(angle * -1.0)

            X_train = np.array(augmented_images)
            y_train = np.array(augmented_angles)

            # X_train = np.array(images)
            # y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)

batch_size = 32
tarin_generator = generator(train_samplses, batch_size=batch_size)
validation_generator = generator(validation_samples, batch_size=batch_size)

# Define the model.

import matplotlib.pyplot as plt

from keras.models import Sequential
from keras.layers import Flatten, Dense, Dropout, Lambda, Cropping2D
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D

model = Sequential()

ch, row, col = 3, 80, 320
#(ch, row, col)
old_shape = (160, 320, 3)
model.add(Lambda(lambda x: x / 255.0 - 0.5, 
    input_shape=old_shape,
    output_shape=old_shape))
model.add(Cropping2D(cropping=((70, 25),(0,0))))
model.add(Convolution2D(24,5,5,subsample=(2,2),activation="relu"))
model.add(Convolution2D(36,5,5,subsample=(2,2),activation="relu"))
model.add(Convolution2D(48,5,5,subsample=(2,2),activation="relu"))
#model.add(MaxPooling2D())
model.add(Convolution2D(64,3,3,activation="relu"))
model.add(Convolution2D(64,3,3,activation="relu"))
#model.add(MaxPooling2D())
model.add(Flatten())
model.add(Dense(100))
model.add(Dense(50))
model.add(Dense(1))

# Train the model.
model.compile(loss='mse', optimizer='adam')
#history_object = model.fit(X_train, y_train, validation_split=0.2, shuffle=True, epochs=10, verbose = 2)
history_object = model.fit_generator(tarin_generator, 
                samples_per_epoch=len(train_samplses),
                validation_data=validation_generator,
                nb_val_samples=len(validation_samples),
                nb_epoch=2)
# Save the model.
model.save('model_many_convs.h5')

# print(history_object.history.keys())
# plt.plot(history_object.history['loss'])
# plt.plot(history_object.history['val_loss'])
# plt.title('model mean squared error loss')
# plt.ylabel('mean squared error loss')
# plt.xlabel('epoch')
# plt.legend(['training set', 'validation set'], loc='upper right')
# plt.show()