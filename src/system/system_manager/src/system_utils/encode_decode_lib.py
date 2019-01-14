
import numpy as np
import simplejson as json

from nautilus_planner.environment.map import Map
from nautilus_planner.environment.thing import Thing
from nautilus_planner.environment.robot import Robot

from nautilus_planner.allocator.allocator import SegmentNode

TYPES = {
  "Thing": Thing,
  "Robot": Robot,
  "Map": Map,
  "SegmentNode": SegmentNode
}

# Encode / Decode classes

class CustomEncoder( json.JSONEncoder ):
  """
  Custom Encoder
  """

  def default(self, obj):

    if isinstance( obj, tuple( TYPES.values() ) ):
      key = "__%s__" % obj.__class__.__name__
      return { key: obj.data }

    return json.JSONEncoder.default(self, obj)

class CustomDecoder( json.JSONDecoder ):
  """
  Custom Decoder
  """

  def decode_dict(self, data_dict):
    sub_key, sub_value = data_dict.items()[0]

    if sub_key.startswith("__") :
      class_name = sub_key.strip("_")
      return TYPES[class_name].from_dict( sub_value )

    return None

  def decode(self, s):
    dict_result = json.JSONDecoder().decode(s)

    for key in dict_result:

      value = dict_result[key]

      if type(value) is list:
        dict_result[key] = []

        for item in value:
          if type(item) is dict:

            data = self.decode_dict( item )

            if data is not None:
              dict_result[key].append( data )
              continue

            for sub_key in item.keys():
              if type(item[sub_key]) is dict:
                item[sub_key] = self.decode_dict( item[sub_key] )

          dict_result[key].append( item )

      if type(value) is dict:
        dict_result[key] = self.decode_dict( value )

    return dict_result

# Encode / Decode methods

def encode_data(data):
  return json.dumps(data, cls=CustomEncoder)

def decode_data(data):
  return json.loads(data, cls=CustomDecoder)
