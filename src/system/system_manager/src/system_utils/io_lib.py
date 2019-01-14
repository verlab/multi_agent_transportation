
import simplejson as json
import cPickle as pickle

def save_object(obj, filename):
  """
  Save a object to a file

  Params:

    obj      : Object to be saved
    filename : Name of the file where the
               object will be saved
  """

  with open(filename, 'wb') as output:
    pickle.dump(obj, output, pickle.HIGHEST_PROTOCOL)

def load_object(filename):
  """
  Loads a object from a file

  Params:

    filename : Name of the file where the
               object was saved
  """

  with open(filename, 'rb') as input:
    return pickle.load(input)
