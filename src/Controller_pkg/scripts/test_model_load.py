#!/usr/bin/env python3
import tensorflow as tf

model_path = "my_model_keras2.h5"

try:
    model = tf.keras.models.load_model(model_path)
    print("✅ Model loaded successfully.")
    model.summary()  # Optional: show architecture
except Exception as e:
    print("❌ Failed to load model:")
    print(e)