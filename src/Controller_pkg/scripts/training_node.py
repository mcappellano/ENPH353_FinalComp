#!/usr/bin/env python3

import os
import re
import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models, Input
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt

# === Config ===
IMAGE_DIR = "/home/fizzer/ros_ws/src/Track_images_V2"
IMAGE_SIZE = (128, 128)
BATCH_SIZE = 32
EPOCHS = 100
LEARNING_RATE = 1e-4

# === Dataset Preparation ===
def load_dataset(image_dir):
    X_data = []
    Y_data = []
    pattern = re.compile(r"lin([-+]?\d+\.\d+)_ang([-+]?\d+\.\d+)\.jpg")

    for filename in os.listdir(image_dir):
        if not filename.endswith(".jpg"):
            continue

        match = pattern.search(filename)
        if not match:
            print(f"Skipping unmatched filename: {filename}")
            continue

        linear = float(match.group(1))
        angular = float(match.group(2))
        Y_data.append([linear, angular])

        img_path = os.path.join(image_dir, filename)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        img = cv2.resize(img, IMAGE_SIZE)
        img = img.astype("float32") / 255.0
        img = np.expand_dims(img, axis=-1)  # shape (128, 128, 1)

        X_data.append(img)

    return np.array(X_data), np.array(Y_data)

# === Model Definition ===
def build_model():
    model = models.Sequential([
        Input(shape=(128, 128, 1)),
        layers.Conv2D(32, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(128, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(128, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Flatten(),
        layers.Dropout(0.5),
        layers.Dense(512, activation='relu'),
        layers.Dense(2)  # Output layer for [linear, angular]
    ])
    return model

# === Main Training ===
def train():
    print("🔄 Loading dataset...")
    X, Y = load_dataset(IMAGE_DIR)

    print(f"✅ Loaded {X.shape[0]} samples")

    X_train, X_val, Y_train, Y_val = train_test_split(X, Y, test_size=0.2, random_state=42)

    model_path = "velocity_predictor_model.h5"

    if os.path.exists(model_path):
        print(f"📥 Loading existing model from {model_path}")
        model = tf.keras.models.load_model(model_path)
    else:
        print("🧠 Building new model...")
        model = build_model()
        model.compile(
            optimizer=tf.keras.optimizers.Adam(learning_rate=LEARNING_RATE),
            loss='mse',
            metrics=['mae']
        )

    callbacks = [
        tf.keras.callbacks.ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=5, min_lr=1e-6, verbose=1),
        tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=10, restore_best_weights=True)
    ]

    print("🚀 Training...")
    history = model.fit(
        X_train, Y_train,
        validation_data=(X_val, Y_val),
        epochs=EPOCHS,
        batch_size=BATCH_SIZE,
        callbacks=callbacks
    )

    # Save the model
    model.save("velocity_predictor_model.h5")
    print("✅ Model saved to velocity_predictor_model.h5")

    # Plot
    plt.plot(history.history['loss'], label='Train Loss')
    plt.plot(history.history['val_loss'], label='Val Loss')
    plt.xlabel('Epoch')
    plt.ylabel('MSE Loss')
    plt.title('Training History')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    print("📦 TensorFlow is using:", "GPU" if tf.config.list_physical_devices('GPU') else "CPU")
    train()
