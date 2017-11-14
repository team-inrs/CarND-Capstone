from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Activation, Dropout, Flatten, Dense
from keras.applications.resnet50 import ResNet50
from keras.layers import Dense, GlobalAveragePooling2D
import keras

# What the model expects
image_width = 224
image_height = 224
input_shape = (image_width, image_height, 3)

# Define our model
def Model():

    # Use ResNet50
    base_model = ResNet50(include_top=False, weights='imagenet')
    x = base_model.output

    # Add pooling layer    
    x = GlobalAveragePooling2D()(x)

    # Add a fully-connected layer
    x = Dense(1024, activation='relu')(x)
    
    # Add a 4 class layer
    predictions = Dense(4, activation='softmax')(x)

    # Our model
    model = keras.models.Model(inputs=base_model.input, outputs=predictions)

    # Train only our custom layers by freezing ResNet50 layers
    for layer in base_model.layers:
        layer.trainable = False

    return model
    
'''
    model = Sequential()
    model.add(Conv2D(32, (3, 3), input_shape=input_shape))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(32, (3, 3)))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(64, (3, 3)))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Flatten())
    model.add(Dense(64))
    model.add(Activation('relu'))
    model.add(Dropout(0.5))
    model.add(Dense(4))
    model.add(Activation('sigmoid'))
    return model
'''

