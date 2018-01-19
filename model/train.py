import os
import sys
import glob
import argparse

from keras import __version__
from keras.applications.mobilenet import MobileNet, preprocess_input
from keras.models import Model
from keras.layers import Dense, GlobalAveragePooling2D
from keras.preprocessing.image import ImageDataGenerator
from keras.optimizers import SGD
from keras.callbacks import EarlyStopping

EPOCHS = 20
BATCH_SIZE = 32
FC_SIZE = 1024
LAYERS_TO_FREEZE = 30
RESOLUTION = 224

def get_file_count(directory):
    """Get number of files by searching directory recursively"""
    if not os.path.exists(directory):
      return 0
    count = 0
    for r, dirs, _ in os.walk(directory):
      for dr in dirs:
        count += len(glob.glob(os.path.join(r, dr + "/*")))
    return count

def add_new_last_layer(base_model, class_count):
    """Add last layer to the convnet
    Args:
      base_model: keras model excluding top
      class_count: # of classes
    Returns:
      new keras model with last layer
    """
    output = Dense(FC_SIZE, activation='relu')(base_model.output)
    predictions = Dense(class_count, activation='softmax')(output)
    model = Model(inputs=base_model.input, outputs=predictions)
    return model

def freeze_layers(model):
    """Freeze the bottom layers and retrain the remaining top layers.
    Args:
      model: keras model
    """
    for layer in model.layers[:LAYERS_TO_FREEZE]:
        layer.trainable = False
    for layer in model.layers[LAYERS_TO_FREEZE:]:
        layer.trainable = True
    model.compile(optimizer='Adam', loss='categorical_crossentropy', metrics=['accuracy'])
    return model

def create_model(class_count):
    input_shape = (int(FLAGS.resolution), int(FLAGS.resolution), 3)
    alpha = float(FLAGS.width_multiplier)
    base_model = MobileNet(
        input_shape=input_shape,
        weights='imagenet',
        alpha=alpha,
        pooling='avg',
        include_top=False
    )
    model = add_new_last_layer(base_model, class_count)
    return model

def train():
    """Use fine-tuning to train a network on a new dataset"""
    train_count = get_file_count(FLAGS.train_dir)
    class_count = len(glob.glob(FLAGS.train_dir + "/*"))
    val_count = get_file_count(FLAGS.val_dir)
    epochs = int(FLAGS.epochs)
    batch_size = int(FLAGS.batch_size)
    target_size = (int(FLAGS.resolution), int(FLAGS.resolution))

    train_datagen =  ImageDataGenerator(
        preprocessing_function=preprocess_input,
        rotation_range=30,
        width_shift_range=0.2,
        height_shift_range=0.2,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True,
    )
    test_datagen = ImageDataGenerator(
        preprocessing_function=preprocess_input,
        rotation_range=30,
        width_shift_range=0.2,
        height_shift_range=0.2,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True,
    )

    train_generator = train_datagen.flow_from_directory(
        FLAGS.train_dir,
        target_size=target_size,
        batch_size=batch_size
    )

    validation_generator = test_datagen.flow_from_directory(
        FLAGS.val_dir,
        target_size=target_size,
        batch_size=batch_size
    )

    model = create_model(class_count) 
    model = freeze_layers(model)
    early_stopping = EarlyStopping(monitor='val_loss', patience=2)

    model.fit_generator(
        train_generator,
        steps_per_epoch=train_count/batch_size,
        epochs=epochs,
        validation_data=validation_generator,
        validation_steps=val_count/batch_size,
        class_weight='auto',
        callbacks=[early_stopping]
    )

    model.save(FLAGS.output_model_file)


if __name__ == "__main__":
    a = argparse.ArgumentParser()
    a.add_argument("--train_dir")
    a.add_argument("--val_dir")
    a.add_argument("--epochs", default=EPOCHS)
    a.add_argument("--batch_size", default=BATCH_SIZE)
    a.add_argument("--output_model_file", default="mobilenet-ft.model")
    a.add_argument("--width_multiplier", default=1)
    a.add_argument("--resolution", default=RESOLUTION)
    FLAGS = a.parse_args()
    if FLAGS.train_dir is None or FLAGS.val_dir is None:
        a.print_help()
        sys.exit(1)

    if (not os.path.exists(FLAGS.train_dir)) or (not os.path.exists(FLAGS.val_dir)):
        print("directories do not exist")
        sys.exit(1)

    train()
