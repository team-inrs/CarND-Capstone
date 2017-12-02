import tensorflow as tf
import numpy as np
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import ModelCheckpoint, ReduceLROnPlateau
from model import Model, image_width, image_height
from styx_msgs.msg import TrafficLight

# Start
print("Running on TensorFlow " + str(tf.__version__))

# Height and width to convert our images to before feeding in

# Set configuration
train_data_dir = 'train'
validation_data_dir = 'validation'
nb_train_samples = 890
nb_validation_samples = 160
epochs = 50
batch_size = 16

# Compile
model = Model()
model.compile(loss='categorical_crossentropy',
              optimizer='adam',
              metrics=['accuracy'])

# Shear, zoom, and flip training data
train_datagen = ImageDataGenerator(
    rescale=1.0 / 255,
    shear_range=0.2,
    zoom_range=0.2,
    horizontal_flip=True)

# Just rescale color for test data
test_datagen = ImageDataGenerator(rescale=1.0 / 255)

# Predefine classes
classes = ['red', 'yellow', 'green', 'none']
print("Classes: " + str(TrafficLight.RED) + ", " + str(TrafficLight.YELLOW) + ", " + str(TrafficLight.GREEN) + ", " + str(TrafficLight.UNKNOWN) + " = " + str(classes))

# Training data
print("Training:")
train_generator = train_datagen.flow_from_directory(
    train_data_dir,
    classes=classes,
    target_size=(image_width, image_height),
    batch_size=batch_size,
    class_mode='categorical')

# Validation data
print("Validation:")
validation_generator = test_datagen.flow_from_directory(
    validation_data_dir,
    classes=classes,
    target_size=(image_width, image_height),
    batch_size=batch_size,
    class_mode='categorical')
    
# Callbacks
checkpoint = ModelCheckpoint(filepath='./model_checkpoint.h5', verbose=1, save_best_only=True)
lr_reducer = ReduceLROnPlateau(factor=np.sqrt(0.1), cooldown=0, patience=5, min_lr=0.5e-6)
callbacks = [checkpoint, lr_reducer]

# Train
model.fit_generator(
    train_generator,
    steps_per_epoch = nb_train_samples // batch_size,
    validation_data = validation_generator,
    validation_steps = nb_validation_samples // batch_size,
    epochs=epochs,
    callbacks=callbacks)

# Save
model.save_weights('model.h5')


