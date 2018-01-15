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

EPOCHS = 3
BATCH_SIZE = 32
FC_SIZE = 1024
LAYERS_TO_FREEZE = 10
TARGET_SIZE = (224, 224)

def get_file_count(directory):
    """Get number of files by searching directory recursively"""
    if not os.path.exists(directory):
      return 0
    count = 0
    for r, dirs, files in os.walk(directory):
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
    output = base_model.output
    output = GlobalAveragePooling2D()(output)
    output = Dense(FC_SIZE, activation='relu')(output) 
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
    model.compile(optimizer=SGD(lr=0.0001, momentum=0.9), loss='categorical_crossentropy', metrics=['accuracy'])

def create_model(args, target_size, class_count): 
    base_model = MobileNet((target_size[0], target_size[1], 3),weights='imagenet', include_top=False) 
    model = add_new_last_layer(base_model, class_count)
    return model

def train(args):
    """Use fine-tuning to train a network on a new dataset"""
    train_count = get_file_count(args.train_dir)
    class_count = len(glob.glob(args.train_dir + "/*"))
    val_count = get_file_count(args.val_dir)
    epochs = int(args.epochs)
    batch_size = int(args.batch_size)
  
    train_datagen =  ImageDataGenerator(
        preprocessing_function=preprocess_input,
        rotation_range=30,
        width_shift_range=0.2,
        height_shift_range=0.2,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True
    )
    test_datagen = ImageDataGenerator(
        preprocessing_function=preprocess_input,
        rotation_range=30,
        width_shift_range=0.2,
        height_shift_range=0.2,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True
    )

    train_generator = train_datagen.flow_from_directory(
      args.train_dir,
      target_size=TARGET_SIZE,
      batch_size=batch_size,
    )

    validation_generator = test_datagen.flow_from_directory(
      args.val_dir,
      target_size=TARGET_SIZE,
      batch_size=batch_size,
    )

    model = create_model(args, TARGET_SIZE, class_count)

    freeze_layers(model)

    history_ft = model.fit_generator(
      train_generator,
      steps_per_epoch=train_count/epochs,
      epochs=epochs,
      validation_data=validation_generator,
      validation_steps=val_count/epochs,
      class_weight='auto')

    model.save(args.output_model_file)


if __name__=="__main__":
    a = argparse.ArgumentParser()
    a.add_argument("--train_dir")
    a.add_argument("--val_dir")
    a.add_argument("--epochs", default=EPOCHS)
    a.add_argument("--batch_size", default=BATCH_SIZE)
    a.add_argument("--output_model_file", default="mobilenet-ft.model")

    args = a.parse_args()
    if args.train_dir is None or args.val_dir is None:
      a.print_help()
      sys.exit(1)

    if (not os.path.exists(args.train_dir)) or (not os.path.exists(args.val_dir)):
      print("directories do not exist")
      sys.exit(1)

    train(args)