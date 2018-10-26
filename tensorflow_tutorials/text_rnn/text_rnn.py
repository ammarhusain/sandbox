# Import TensorFlow >= 1.9 and enable eager execution
import tensorflow as tf

# Note: Once you enable eager execution, it cannot be disabled.
tf.enable_eager_execution()

import numpy as np
import re
import random
import unidecode
import time


path_to_file  = tf.keras.utils.get_file('shakespeare.txt', 'https://storage.googleapis.com/yashkatariya/shakespeare.txt')

text = unidecode.unidecode(open(path_to_file).read())
# length of text is the number of characters in it
print (len(text))

# unique contains all the unique characters in the file
unique = sorted(set(text))

# creating a mapping from unique characters to indices
char2idx = {u:i for i, u in enumerate(unique)}
idx2char = {i:u for i, u in enumerate(unique)}

# setting the maximum length sentence we want for a single input in characters
max_length = 100

# length of the vocabulary in chars
vocab_size = len(unique)

# the embedding dimension
embedding_dim = 256

# number of RNN (here GRU) units
units = 1024

# batch size
BATCH_SIZE = 64

# buffer size to shuffle our dataset
BUFFER_SIZE = 10000

input_text = []
target_text = []

for f in range(0, len(text)-max_length, max_length):
    inps = text[f:f+max_length]
    targ = text[f+1:f+1+max_length]

    input_text.append([char2idx[i] for i in inps])
    target_text.append([char2idx[t] for t in targ])

print (np.array(input_text).shape)
print (np.array(target_text).shape)

# Create batches and shuffle them using tf.data
dataset = tf.data.Dataset.from_tensor_slices((input_text, target_text)).shuffle(BUFFER_SIZE)
dataset = dataset.apply(tf.contrib.data.batch_and_drop_remainder(BATCH_SIZE))

class Model(tf.keras.Model):
  def __init__(self, vocab_size, embedding_dim, units, batch_size):
    super(Model, self).__init__()
    self.units = units
    self.batch_sz = batch_size

    self.embedding = tf.keras.layers.Embedding(vocab_size, embedding_dim)

    if tf.test.is_gpu_available():
      self.gru = tf.keras.layers.CuDNNGRU(self.units,
                                          return_sequences=True,
                                          return_state=True,
                                          recurrent_initializer='glorot_uniform')
    else:
      self.gru = tf.keras.layers.GRU(self.units,
                                     return_sequences=True,
                                     return_state=True,
                                     recurrent_activation='sigmoid',
                                     recurrent_initializer='glorot_uniform')

    self.fc = tf.keras.layers.Dense(vocab_size)

  def call(self, x, hidden):
    x = self.embedding(x)

    # output shape == (batch_size, max_length, hidden_size)
    # states shape == (batch_size, hidden_size)

    # states variable to preserve the state of the model
    # this will be used to pass at every step to the model while training
    output, states = self.gru(x, initial_state=hidden)


    # reshaping the output so that we can pass it to the Dense layer
    # after reshaping the shape is (batch_size * max_length, hidden_size)
    output = tf.reshape(output, (-1, output.shape[2]))

    # The dense layer will output predictions for every time_steps(max_length)
    # output shape after the dense layer == (max_length * batch_size, vocab_size)
    x = self.fc(output)

    return x, states

model = Model(vocab_size, embedding_dim, units, BATCH_SIZE)

optimizer = tf.train.AdamOptimizer()

# using sparse_softmax_cross_entropy so that we don't have to create one-hot vectors
def loss_function(real, preds):
    return tf.losses.sparse_softmax_cross_entropy(labels=real, logits=preds)

# Training step

EPOCHS = 30

for epoch in range(EPOCHS):

    start = time.time()
    # initializing the hidden state at the start of every epoch
    hidden = model.reset_states()

    for (batch, (inp, target)) in enumerate(dataset):
          with tf.GradientTape() as tape:
              # feeding the hidden state back into the model
              # This is the interesting step
              predictions, hidden = model(inp, hidden)
              # reshaping the target because that's how the
              # loss function expects it
              target = tf.reshape(target, (-1,))
              loss = loss_function(target, predictions)

          grads = tape.gradient(loss, model.variables)
          optimizer.apply_gradients(zip(grads, model.variables), global_step=tf.train.get_or_create_global_step())

          if batch % 100 == 0:
              print ('Epoch {} Batch {} Loss {:.4f}'.format(epoch+1,
                                                            batch,
                                                            loss))
    print ('Epoch {} Loss {:.4f}'.format(epoch+1, loss))
    print('Time taken for 1 epoch {} sec\n'.format(time.time() - start))
