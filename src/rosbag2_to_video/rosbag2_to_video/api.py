# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from genericpath import isdir
import pathlib
import sys
from typing import Any
from typing import Callable
from typing import Tuple
from typing import TYPE_CHECKING

import cv2
from cv_bridge import CvBridge

from rclpy.serialization import deserialize_message
import rclpy.time
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

if TYPE_CHECKING:
    from argparse import ArgumentParser
    import numpy as np

captions = [
    {
        "caption": "The video shows a long hallway with a shiny floor and a glass "
        "wall on the right. There are two people in the hallway. One "
        "person is standing near the glass wall, and the other person is "
        "walking towards the camera. The hallway is well-lit, and there "
        "are signs on the walls. The signs are written in Chinese, and "
        "they are blue and white in color. The overall atmosphere of the "
        "video is calm and quiet.",
        "position": [-3.1587595353988243, -0.2795824271143262, 0.0],
        "theta": 3.1101523127904254,
        "time": 1741509137.551421,
    },
    {
        "caption": "The video shows a long, empty hallway with a shiny floor. The "
        "walls are white, and there are glass doors on both sides. The "
        "doors have blue and white signs on them, and there is a large "
        'sign in the center of the hallway that reads "Institute of '
        'Advanced Technology". The hallway is well-lit, and there is a '
        "person standing in the distance.",
        "position": [-5.1715615793781655, -0.2073964348530434, 0.0],
        "theta": 3.1113985682694802,
        "time": 1741509140.1565418,
    },
    {
        "caption": "The video shows a long, empty hallway with a shiny, reflective "
        "floor. The walls are white, and there are several informational "
        "posters and signs on them. The most prominent sign is a large "
        'blue sign with white text that reads "Institute of Advanced '
        'Clinical". The hallway is well-lit, and there are no people or '
        "objects in the frame. The overall atmosphere is calm and quiet.",
        "position": [-6.34580597502692, -0.18455595670929584, 0.0],
        "theta": 3.111363445174062,
        "time": 1741509143.3622212,
    },
    {
        "caption": "The video shows a long hallway with a shiny floor and white "
        "walls. The hallway leads to a large glass door at the end. On the "
        'wall next to the door, there is a sign that reads "Institute of '
        'Advanced Computing". The sign is blue and white, with a picture '
        "of a city skyline in the background. The lighting in the hallway "
        "is bright, and there are no people or objects in the hallway.",
        "position": [-7.341030767339975, -0.16319671875620623, 0.0],
        "theta": -1.5693705659422303,
        "time": 1741509146.667862,
    },
    {
        "caption": "The video shows a long corridor with a shiny floor. On the left "
        "side of the corridor, there are three white boards with black "
        "text and diagrams. The right side of the corridor has a large "
        "blue wall with white text and a picture of a city skyline. The "
        'text on the wall reads "Institute of Advanced Computing". The '
        "lighting in the corridor is bright, and the floor reflects the "
        "light, giving it a glossy appearance.",
        "position": [-8.29596771625483, -0.15661883643734995, 0.0],
        "theta": -3.115418533130123,
        "time": 1741509149.9732616,
    },
    {
        "caption": "The video shows a large blue sign with white text that reads "
        '"Institute of Advanced Computing." The sign is located in a room '
        "with a gray floor and white walls. There are several white boards "
        "with information and diagrams on the left side of the sign. The "
        "sign is illuminated, and the lighting in the room casts a blue "
        "hue on the sign.",
        "position": [-9.310395251923063, -0.21115385938648606, 0.0],
        "theta": -3.017817128372818,
        "time": 1741509153.477801,
    },
    {
        "caption": "The video shows a person walking past a wall with a large blue "
        "and white illustration of various buildings and structures. The "
        "person is wearing jeans and a blue jacket. The floor is a light "
        "grey color, and there are several informational posters on the "
        "wall.",
        "position": [-10.270538120915573, -0.37061653705551895, 0.0],
        "theta": -2.8728886488644663,
        "time": 1741509156.582681,
    },
    {
        "caption": "The video shows a corridor with a grey floor and walls. There are "
        "several posters on the walls, some of which are lit up with blue "
        "lights. The posters contain text and images, but the content is "
        "not clear. The corridor appears to be empty with no people "
        "visible.",
        "position": [-11.101338565547097, -0.7883949961239909, 0.0],
        "theta": -2.475618560448736,
        "time": 1741509159.587141,
    },
    {
        "caption": "The video shows a long, narrow hallway in a building. The floor "
        "is made of gray tiles, and there are several potted plants placed "
        "along the walls. The walls are adorned with various posters and "
        "signs. A man in a blue shirt and black pants is seen walking down "
        "the hallway, carrying a black bag. He passes by a glass wall that "
        "reflects the interior of the building. The lighting in the "
        "hallway is bright, and the overall atmosphere is calm and quiet.",
        "position": [-11.688403220544734, -1.6897495656035717, 0.0],
        "theta": -1.8170410433687845,
        "time": 1741509162.5908604,
    },
    {
        "caption": "The video shows a person standing in a hallway, talking on a cell "
        "phone. The person is wearing a blue shirt and black pants. The "
        "hallway is lined with plants and has a glass wall on one side. "
        "The floor is made of tiles, and there is a trash can next to the "
        "person. The person appears to be engaged in a conversation, as "
        "they are holding the phone to their ear.",
        "position": [-11.769865728464616, -2.5619386980720846, 0.0],
        "theta": -1.5125245148002728,
        "time": 1741509166.4404058,
    },
    {
        "caption": "The video shows a long, narrow hallway with a shiny floor. On the "
        "left side of the hallway, there is a trash can and a potted "
        "plant. On the right side, there is a glass wall with a reflection "
        "of a person. The person is wearing a blue shirt and black pants. "
        "The hallway appears to be well-lit and clean.",
        "position": [-11.777471949472567, -3.593169256571869, 0.0],
        "theta": -1.5410059171532235,
        "time": 1741509168.798681,
    },
    {
        "caption": "The video shows a long, narrow hallway with a shiny, light blue "
        "floor. The walls are adorned with various posters and signs, and "
        "there are several potted plants placed along the corridor. The "
        "hallway appears to be in a modern building, possibly an office or "
        "a hospital. The lighting is bright, and the overall atmosphere is "
        "clean and professional.",
        "position": [-11.796558199134246, -4.785656915386007, 0.0],
        "theta": -1.5539919310822587,
        "time": 1741509172.504401,
    },
    {
        "caption": "The video shows a long hallway with a shiny floor and white "
        "walls. There are three potted plants placed on the floor, and the "
        "walls are adorned with posters and signs. The hallway is "
        "well-lit, and there is a green exit sign visible.",
        "position": [-11.775640241146128, -5.7988658984808765, 0.0],
        "theta": -1.559411389805936,
        "time": 1741509175.6097212,
    },
    {
        "caption": "The video shows a long, narrow hallway with a light blue floor. "
        "There are three potted plants placed on the left side of the "
        "hallway. The walls are adorned with various posters and charts. "
        "The hallway is well-lit, and there is a green exit sign visible "
        "in the distance.",
        "position": [-11.80352770828652, -6.848705232369983, 0.0],
        "theta": -1.5671494465784988,
        "time": 1741509178.614841,
    },
    {
        "caption": "The video shows a hallway with a blue floor and white walls. "
        "There are two potted plants on either side of the hallway. A "
        "person is walking down the hallway, wearing a blue shirt and "
        "black pants. The person is carrying a white bag.",
        "position": [-11.845862651602355, -7.935400296057509, 0.0],
        "theta": -1.545665036740207,
        "time": 1741509181.6188207,
    },
    {
        "caption": "The video shows a person in a blue shirt and black pants bending "
        "over to pick up a blue bucket from the floor. The person then "
        "stands up and walks away down a hallway with a green exit sign "
        "visible on the wall. The hallway is lined with white walls and "
        "has a blue floor. There are posters on the walls and a water "
        "cooler on the right side.",
        "position": [-11.859218540898478, -8.93366062793479, 0.0],
        "theta": -1.55745075975433,
        "time": 1741509184.6226401,
    },
    {
        "caption": "The video shows a long hallway with a green digital clock on the "
        "wall. The clock reads 10:10. The floor is a light blue color, and "
        "there are two trash cans on either side of the hallway. The walls "
        "are white, and there are posters on them. The hallway is "
        "well-lit, and there are doors at the end of the hallway.",
        "position": [-11.931632282891103, -10.138370066284685, 0.0],
        "theta": -1.5917407524223344,
        "time": 1741509187.82754,
    },
    {
        "caption": "The video shows a corridor with a green exit sign on the wall. "
        "The sign is rectangular with a green background and white text. "
        "The corridor has white walls and a gray floor. There are doors on "
        "both sides of the corridor.",
        "position": [-11.96962920124866, -11.250598110702068, 0.0],
        "theta": -1.4939146321257264,
        "time": 1741509191.3220005,
    },
    {
        "caption": "The video shows a corridor with a green exit sign on the wall. "
        "The sign is rectangular with a white background and a green "
        "border. The sign is located on the right side of the corridor. "
        "The floor of the corridor is gray and the walls are white. There "
        "are no other objects or people visible in the video.",
        "position": [-11.6840729141317, -12.192826709847983, 0.0],
        "theta": -0.924155313696354,
        "time": 1741509194.3712,
    },
    {
        "caption": "The video shows a long, narrow hallway with a shiny floor and "
        "white walls. There are several doors on both sides of the "
        "hallway, and the ceiling is equipped with fluorescent lights. The "
        "hallway appears to be empty, with no people or objects in sight. "
        "The walls are adorned with posters and signs, but the text is not "
        "legible. The overall atmosphere of the hallway is quiet and calm.",
        "position": [-10.776012477755938, -12.59924899448396, 0.0],
        "theta": -0.10670111410729352,
        "time": 1741509197.6416993,
    },
    {
        "caption": "The video shows a long, narrow hallway with a shiny, grey floor. "
        "The walls are white and adorned with various posters and signs. "
        "The hallway is well-lit, with fluorescent lights on the ceiling. "
        "There are doors on both sides of the hallway, some of which are "
        "open. The hallway appears to be in a hospital or medical "
        "facility, as suggested by the presence of emergency exit signs.",
        "position": [-9.362314891547694, -12.594700518410825, 0.0],
        "theta": 0.07627820023147477,
        "time": 1741509201.6471589,
    },
    {
        "caption": "The video shows a long, narrow hallway with a shiny, gray floor. "
        "The walls are white and adorned with framed pictures and posters. "
        "The hallway is well-lit, with fluorescent lights on the ceiling. "
        "There are several doors on either side of the hallway, some of "
        "which are open. The hallway appears to be in a modern building, "
        "possibly an office or institutional building.",
        "position": [-8.164169679119329, -12.606166746556806, 0.0],
        "theta": 0.02889572859300909,
        "time": 1741509204.9506388,
    },
    {
        "caption": "The video shows a long, narrow hallway with a shiny gray floor. "
        "The walls are white, and there are several doors on either side. "
        "The ceiling is white with exposed pipes. There is a green exit "
        "sign on the wall. The hallway is empty, and there are no people "
        "or objects in the frame.",
        "position": [-6.94813791026034, -12.598437644681756, 0.0],
        "theta": 0.02453994637454822,
        "time": 1741509208.356879,
    },
    {
        "caption": "The video shows a long hallway with a gray floor and white walls. "
        "There are several doors on the right side of the hallway, and a "
        "green exit sign is visible on the right wall. The hallway is "
        "well-lit with fluorescent lights, and there are potted plants "
        "placed at regular intervals along the hallway. The hallway "
        "appears to be in a modern building, possibly an office or "
        "hospital.",
        "position": [-5.750973331941005, -12.650694076261146, 0.0],
        "theta": -0.011522927417822616,
        "time": 1741509211.3613544,
    },
    {
        "caption": "The video shows a long, narrow hallway with a shiny, blue-gray "
        "floor. The walls are white, and there are several doors on either "
        "side. Some of the doors are open, revealing rooms or offices "
        "inside. The ceiling is high, with exposed pipes and wires running "
        "across it. There are also several potted plants placed along the "
        "hallway, adding a touch of greenery to the space. The lighting is "
        "bright, and the overall atmosphere is clean and professional.",
        "position": [-4.511408641128176, -12.711842650950825, 0.0],
        "theta": -0.0022364195275326694,
        "time": 1741509214.6655102,
    },
    {
        "caption": "The video shows a long, narrow hallway with a blue-gray floor. "
        "The walls are white, and there are several potted plants placed "
        "along the hallway. The ceiling is equipped with fluorescent "
        "lights, and there are signs hanging from the ceiling. The hallway "
        "appears to be in a modern office building.",
        "position": [-3.116257416901002, -12.669572131227435, 0.0],
        "theta": 0.05302186651059184,
        "time": 1741509218.17031,
    },
    {
        "caption": "The video shows a long, narrow hallway with a blue floor and "
        "white walls. On the left side of the hallway, there are several "
        "white cubicles with doors closed. The hallway is well-lit with "
        "fluorescent lights. There are also several potted plants placed "
        "along the hallway, adding a touch of greenery to the space. The "
        "hallway appears to be in a modern office building.",
        "position": [-1.7486342001135078, -12.690206879572823, 0.0],
        "theta": 0.03068211796034593,
        "time": 1741509221.17499,
    },
    {
        "caption": "The video shows a long, empty office corridor with a shiny floor. "
        "The walls are white, and there are several doors on the right "
        "side. The ceiling has exposed pipes and lights. There are no "
        "people or objects in the corridor.",
        "position": [-0.29152630542222596, -12.7055470276015, 0.0],
        "theta": 0.1356192070296166,
        "time": 1741509224.42915,
    },
    {
        "caption": "The video shows a long, narrow hallway with a gray floor and "
        "white walls. There are several gray cubicles on the left side of "
        "the hallway, and a few people can be seen walking in the "
        "distance. The hallway has fluorescent lights on the ceiling and a "
        "green exit sign on the right side.",
        "position": [1.1444903682360723, -12.506636025356503, 0.0],
        "theta": 0.25413187612415017,
        "time": 1741509227.48355,
    },
    {
        "caption": "The video shows a long hallway with gray flooring and white "
        "walls. There are several gray cubicles along the sides of the "
        "hallway. The cubicles have white doors and are equipped with "
        "black office chairs. There are also signs on the walls indicating "
        "no smoking and no food or drinks allowed. The hallway is well-lit "
        "with fluorescent lights.",
        "position": [2.2646977905065104, -12.072062089909792, 0.0],
        "theta": 0.5807607275822995,
        "time": 1741509230.48835,
    },
    {
        "caption": "The video shows a long hallway with a series of cubicles on "
        "either side. The cubicles are made of white panels and have a "
        "gray carpeted floor. The hallway is well-lit with fluorescent "
        "lights. There are no people or objects in the hallway, and the "
        "cubicles appear to be empty.",
        "position": [2.950226347561933, -11.216696493791417, 0.0],
        "theta": 1.1787454346954178,
        "time": 1741509233.6940699,
    },
    {
        "caption": "The video shows a long hallway with a series of cubicles on "
        "either side. The cubicles are made of white panels and have a "
        "door at the front. The floor is gray and there are several boxes "
        "stacked against the wall. The hallway is well-lit with "
        "fluorescent lights hanging from the ceiling.",
        "position": [3.09571291260057, -10.069769748682335, 0.0],
        "theta": 1.6668029759116088,
        "time": 1741509236.79827,
    },
    {
        "caption": "The video shows a long hallway with several cubicles on either "
        "side. The cubicles are empty and have a gray color. There are two "
        "computer monitors in the hallway, one on top of a box and the "
        "other on a desk. The floor is gray and there are fluorescent "
        "lights on the ceiling.",
        "position": [3.097455985322532, -8.734194730067768, 0.0],
        "theta": 1.6316580229561457,
        "time": 1741509239.90327,
    },
    {
        "caption": "The video shows a long hallway with gray cubicles on either side. "
        "The floor is gray and there are fluorescent lights on the "
        "ceiling. There are a few chairs and desks visible in the "
        "cubicles. The walls are white and there is a black office chair "
        "visible in the foreground.",
        "position": [3.0771823456091902, -7.397132137333355, 0.0],
        "theta": 1.6338118412719134,
        "time": 1741509242.906921,
    },
    {
        "caption": "The video shows a long hallway with gray cubicles on either side. "
        "The floor is gray and there are white ceiling lights. The "
        "cubicles have white doors and some have black chairs in front of "
        "them. There is a trash can in the hallway.",
        "position": [3.117927943916154, -5.958705742264263, 0.0],
        "theta": 1.6076545992645894,
        "time": 1741509245.911721,
    },
    {
        "caption": "The video shows a long hallway with gray carpeting and a series "
        "of cubicles on either side. The cubicles are white with gray "
        "doors and have a small trash can in front of them. The hallway is "
        "well-lit with fluorescent lights on the ceiling. There is no one "
        "visible in the hallway, and the cubicles are empty.",
        "position": [3.091944869084086, -4.553228898652327, 0.0],
        "theta": 1.6173001742563158,
        "time": 1741509248.916601,
    },
    {
        "caption": "The video shows a large room with a gray carpeted floor. In the "
        "center of the room, there is a large gray partition that is being "
        "pushed open by a person. The partition is made of metal and has a "
        "handle on the side. The room is well-lit with several lights on "
        "the ceiling. In the background, there are several desks and "
        "chairs, some of which are occupied by people. The walls of the "
        "room are white and there are several windows that let in natural "
        "light.",
        "position": [3.067622234031837, -3.1449783493821983, 0.0],
        "theta": 1.639445364117515,
        "time": 1741509252.1221814,
    },
    {
        "caption": "The video shows a large office space with gray cubicles. The "
        "cubicles are made of a combination of gray and white materials, "
        "and they have a modern design. There are several people in the "
        "office, some of whom are sitting at their desks, while others are "
        "standing. The office has a spacious layout with ample room for "
        "movement. The lighting in the office is bright, and the overall "
        "atmosphere appears to be professional and organized.",
        "position": [3.032701079088605, -1.8452598222866547, 0.0],
        "theta": 1.7430535287035762,
        "time": 1741509255.6278815,
    },
    {
        "caption": "The video shows a long hallway with a shiny floor, leading to a "
        "room with glass walls. There are several people in the room, and "
        "one person is sitting at a desk with a computer. The walls are "
        "white, and there are several cabinets and drawers along the "
        "hallway. The room is well-lit, and there are blue trash cans in "
        "the corner.",
        "position": [2.5131438655562928, -0.8893918477381537, 0.0],
        "theta": 2.529242047372439,
        "time": 1741509259.334414,
    },
    {
        "caption": "The video shows a long, narrow hallway with a shiny, gray floor. "
        "The walls are white, and there are several doors on the left "
        "side. The hallway leads to a glass door at the end, which has "
        "Chinese characters on it. There is a person sitting at a desk in "
        "the background, and another person is standing in front of the "
        "glass door. The lighting in the hallway is bright, and the "
        "overall atmosphere is calm and quiet.",
        "position": [1.1082683089340437, -0.643219940611624, 0.0],
        "theta": -2.227676491132742,
        "time": 1741509262.7391214,
    },
    {
        "caption": "The video shows a long, narrow hallway with a shiny floor. The "
        "walls are white, and there are glass doors on both sides. The "
        "doors have blue and white signs on them. The hallway leads to a "
        'room with a blue and white sign that reads "医院" which translates '
        'to "hospital." The lighting in the hallway is bright, and there '
        "are no people or objects in the hallway.",
        "position": [-0.24879805479123232, -0.7247941481640233, 0.0],
        "theta": -3.0917244253342786,
        "time": 1741509266.7444015,
    },
    {
        "caption": "The video shows a long, empty hallway with a shiny, gray floor. "
        "The walls are white, and there are glass doors on both sides of "
        "the hallway. The doors have blue and white signs on them. The "
        "hallway leads to a room with a red door at the end. There are no "
        "people or objects in the hallway.",
        "position": [-0.5360129010794659, -0.7481793278072325, 0.0],
        "theta": -3.097225419763323,
        "time": 1741509270.1494808,
    },
    {
        "caption": "The video shows a long, empty hallway with a shiny, grey floor. "
        "The walls are white, and there are glass doors on the right side "
        "of the hallway. The ceiling is fitted with fluorescent lights. "
        "The hallway leads to a room with a red door at the end.",
        "position": [-0.5360129010794659, -0.7481793278072325, 0.0],
        "theta": -3.097225419763323,
        "time": 1741509273.252992,
    },
    {
        "caption": "The video shows a long, empty hallway with a shiny, gray floor. "
        "The walls are white, and there are several glass doors on the "
        "right side. The ceiling has exposed pipes and wires. The hallway "
        "leads to a brightly lit room at the end, which has a blue and "
        "white sign on the wall.",
        "position": [-0.5360129010794659, -0.7481793278072325, 0.0],
        "theta": -3.097225419763323,
        "time": 1741509276.257592,
    },
    {
        "caption": "The video shows a long, empty hallway with a shiny floor. The "
        "walls are white, and there are glass doors on the right side. The "
        "ceiling has exposed pipes and lights. The hallway leads to a room "
        "with a blue wall and a red door.",
        "position": [-0.5360129010794659, -0.7481793278072325, 0.0],
        "theta": -3.097225419763323,
        "time": 1741509279.3623118,
    },
    {
        "caption": "The video shows a long, narrow hallway with a gray floor and "
        "white walls. The hallway is empty, with no people or objects in "
        "sight. The walls are adorned with glass doors and windows, "
        "reflecting the light and creating a bright and airy atmosphere. "
        "The ceiling is fitted with exposed pipes and wires, adding an "
        "industrial touch to the space. The hallway leads to a brightly "
        "lit room at the end, where a blue sign with white text is "
        'visible. The sign reads "办公室", which translates to "Office" in '
        "English. The overall ambiance of the video is calm and serene, "
        "with a sense of quiet anticipation for what lies beyond the "
        "office.",
        "position": [-0.5360129010794659, -0.7481793278072325, 0.0],
        "theta": -3.097225419763323,
        "time": 1741509282.366512,
    },
]


def cv2_video_writer_fourcc(codec_str: str) -> int:
    """Validate the user provided video codec string and return it as fourcc."""
    if len(codec_str) != 4:
        raise ValueError(f'codecs should be specified using fourcc, got "{codec_str}"')
    try:
        return cv2.VideoWriter.fourcc(*codec_str)
    except Exception:
        raise ValueError(f'"{codec_str}" is not a valid fourcc codec')

def list_available_codecs():
    """List available video codecs supported by OpenCV."""
    fourcc_list = [
        "DIVX", "MJPG", "XVID", "MP4V", "H264", "AVC1", "HEVC", "VP8"
    ]
    print("Available codecs:")
    for codec in fourcc_list:
        try:
            fourcc = cv2.VideoWriter_fourcc(*codec)
            print(f"{codec}: {fourcc}")
        except Exception:
            pass



def add_arguments_to_parser(argparser: 'ArgumentParser'):
    """Define command line arguments for the bag to video tool."""
    argparser.add_argument(
        'bagfile',
        help='Path to the bag'
    )
    argparser.add_argument(
        '-t',
        '--topic',
        required=True,
        help=(
            'Name of the image topic (currently only supports sensor_msgs/msg/Image or '
            'sensor_msgs/msg/CompressedImage types)')
    )
    argparser.add_argument(
        '-o',
        '--output',
        required=True,
        help='Output filename. If no extension is provided, .mp4 is added.'
    )
    argparser.add_argument('--fps', type=float, required=True, help='Output frames per second')
    argparser.add_argument(
        '--storage-id',
        type=str,
        default='sqlite3',
        help='Rosbag2 storage id. If a bag folder is provided, this is ignored.'
    )
    codec_group = argparser.add_mutually_exclusive_group()
    codec_group.add_argument(
        '--codec', type=cv2_video_writer_fourcc, default='avc1',
        help=(
            'Video codec fourcc. List of fourcc can be seen in http://mp4ra.org/#/codecs. '
            'ffmpeg does not follow exactly that list, use --codec-dialog to see what is available'
        ))
    argparser.add_argument(
        '--codec-dialog', action='store_true',
        help=(
            'Open video codec dialog. In some platforms, this will only print the available '
            'fourcc options (which depend on the file format chosen).'))



class CommandInputError(ValueError):
    """Error raised when provided command line arguments are not valid."""

    def __init__(self, msg: str):
        super().__init__(msg)


def get_stamp_from_image_msg(image_msg) -> float:
    """Convert timestamp in msg from nanoseconds to seconds."""
    stamp = rclpy.time.Time.from_msg(image_msg.header.stamp).nanoseconds
    stamp = stamp / 1e9
    return stamp


def get_topic_type(topic_name: str, topics_and_types) -> str:
    """Get the topic type from the topic name and the topic information in the bag."""
    try:
        topic_type = next(x for x in topics_and_types if x.name == topic_name).type
    except StopIteration:
        raise CommandInputError(
            f'Topic {topic_name} was not recorded in the bagfile')
    if topic_type not in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'):
        raise CommandInputError(
            'topic type should be sensor_msgs/msg/Image or '
            f'sensor_msgs/msg/CompressedImage, got {topic_type}')
    return topic_type


class SequentialImageBagReader:
    """Reader of images from a bagfile source sequentially"""

    def __init__(self, bag_reader: rosbag2_py.SequentialReader, topic_name: str):
        """
        Create image bagfile reader.

        :param bag_reader: rosbag2_py sequential reader instance.
        :param topic_name: topic from where to read the images in the bagfile.
        """
        self._bag_reader: rosbag2_py.SequentialReader = bag_reader
        self._cvbridge: CvBridge = CvBridge()
        self._topic_name: str = topic_name
        self._topic_type: str = get_topic_type(topic_name, bag_reader.get_all_topics_and_types())
        self._msg_type = get_message(self._topic_type)
        self._msg_to_cv2: Callable[[Any], 'np.ndarray']
        if self._topic_type == 'sensor_msgs/msg/Image':
            self._msg_to_cv2 = lambda msg: self._cvbridge.imgmsg_to_cv2(msg, 'bgr8')
        elif self._topic_type == 'sensor_msgs/msg/CompressedImage':
            self._msg_to_cv2 = lambda msg: self._cvbridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
        self._bag_reader.set_filter(storage_filter)
    
    def next(self) -> Tuple['np.ndarray', float]:
        """Return next image and its timestamp."""
        _, data, _ = self._bag_reader.read_next()    
        image_msg = deserialize_message(data, self._msg_type)
        return self._msg_to_cv2(image_msg), get_stamp_from_image_msg(image_msg)
    
    def has_next(self):
        """Return true if there is at least one more message to read."""
        return self._bag_reader.has_next()


class SequentialVideoWriter:
    """Create videos from images provided sequentially."""

    def __init__(
        self,
        cv_video_writer: cv2.VideoWriter,
        first_cv_image: 'np.ndarray',
        start_stamp: float,
        fps: float,
    ):
        """
        Create a video writer.

        :param cv_video_writer: A cv2.VideoWriter instance, already opened.
        :param first_cv_image: First image to write in the video.
        :param start_stamp: Timestamp of the first image.
        :param fps: Fps used to record the video.
        """
        self._fps: float = fps
        self._cv_video_writer: cv2.VideoWriter = cv_video_writer
        self._cv_video_writer.write(first_cv_image)
        self._images_processed: int = 1
        self._frame_count: int = 1
        self._images_skipped: int = 0
        self._start_stamp: float = start_stamp
        self._last_image: 'np.ndarray' = first_cv_image
        self._last_image_written_once: bool = True
    
    def add_frame(self, cv_image: 'np.ndarray', stamp: float):
        """Add a frame to the video, given an image and its timestamp."""

        self._images_processed += 1
        t_from_start = stamp - self._start_stamp

        # accept jitter up to 0.5/fps
        if t_from_start < (float(self._frame_count) - 0.5) / self._fps:
            self._last_image = cv_image
            if not self._last_image_written_once:
                self._images_skipped += 1
                print(
                    'video fps is too low compared to image publish rate, skipping one message',
                    file=sys.stderr)
            self._last_image_written_once = False
            return
        current_image_frame_index = int(round(t_from_start * self._fps))
        repeat_last_image = current_image_frame_index - self._frame_count
        for _ in range(repeat_last_image):
            self._cv_video_writer.write(self._last_image)
        self._cv_video_writer.write(cv_image)
        self._last_image = cv_image
        if not self._last_image_written_once and repeat_last_image == 0:
            self._images_skipped += 1
            print(
                'video fps is too low compared to image publish rate, skipping one message',
                file=sys.stderr)
        self._last_image_written_once = True
        self._frame_count += repeat_last_image + 1
    
    def close(self):
        """Close the video writer."""
        if not self._last_image_written_once:
            self._cv_video_writer.write(self._last_image)
        self._cv_video_writer.release()
    
    @property
    def images_processed(self) -> int:
        """
        Get the number of images that were provided.

        Images that were skipped and not written are also counted.
        """
        return self._images_processed

    @property
    def frames_written(self) -> int:
        """
        Number of frames written to the video.
        """
        return self._frame_count
    
    @property
    def images_skipped(self) -> int:
        """Number of images that were skipped and not written to the video."""
        return self._images_skipped


def create_sequential_image_bag_reader(
    bag_path: str, storage_id: str, topic_name: str
) -> SequentialImageBagReader:
    """
    Create a SequentialImageBagReader instance.

    :param bag_path: Path to the bagfile (folder with metadata or file).
    :param storage_id: Storage id of the bagfile.
    :param topic_name: Name of the image topic.
    """
    if pathlib.Path(bag_path).is_dir():
        # load storage id from metadata.yaml
        storage_options = rosbag2_py.StorageOptions(uri=bag_path)
    else:
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    # TODO(jacobperron): Shouldn't we be able to infer serialization format from metadaya.yaml?
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    bag_reader = rosbag2_py.SequentialReader()
    bag_reader.open(storage_options, converter_options)

    if not bag_reader.has_next():
        raise CommandInputError('empty bag file')
    return SequentialImageBagReader(bag_reader, topic_name)


def create_sequential_video_writer(
    output_path: str,
    codec: int,
    fps: float,
    first_cv_image: 'np.ndarray',
    start_stamp: float
):
    """
    Create a SequentialVideoWriter instance.

    :param output_path: Path of the video to be created.
    :param codec: fourcc of the codec to be used.
    :param fps: Video frame per second to be used.
    :param first_cv_image: First image to write to the video.
        Video width and height are got from here.
    :param start_stamp: Timestamp of the first image.
    """
    height, width, _ = first_cv_image.shape
    cv_video_writer = cv2.VideoWriter()
    success = cv_video_writer.open(
        output_path,
        cv2.CAP_FFMPEG,
        codec,
        fps,
        (width, height))

    if not success:
        print(f"Failed to open codec {codec}. Trying alternative codecs.", file=sys.stderr)
        # Attempt using alternative codecs
        alternative_codecs = ['MJPG', 'XVID', 'MP4V']
        for alt_codec in alternative_codecs:
            alt_fourcc = cv2.VideoWriter_fourcc(*alt_codec)
            success = cv_video_writer.open(
                output_path,
                cv2.CAP_FFMPEG,
                alt_fourcc,
                fps,
                (width, height)
            )
            if success:
                print(f"Successfully opened with codec: {alt_codec}")
                codec = alt_fourcc
                break
        if not success:
            raise CommandInputError(f'Failed to open file {output_path} with any codec')
    return SequentialVideoWriter(cv_video_writer, first_cv_image, start_stamp, fps)



def add_caption_to_frame(frame, caption_data):
    """Add caption text to the frame with a white background, black text, and transparency, and print position and orientation in the top-right corner."""
    
    # Check if caption_data is a dictionary
    if not isinstance(caption_data, dict):
        raise TypeError("Expected caption_data to be a dictionary")

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_color = (0, 0, 0)  # Black text
    font_red_color = (255,0,0) #red
    line_type = 2
    max_line_length = frame.shape[1] - 100  # Max line length (screen width minus margins)
    background_color = (255, 255, 255)  # White background
    alpha = 0.6  # Transparency level of the background rectangle

    # Extract position and theta from the caption data
    pos = caption_data.get("position", [0, 0, 0])  # Default to [0, 0, 0] if not found
    theta = caption_data.get("theta", 0)  # Default to 0 if not found
    caption = caption_data.get("caption", "")  # Default to empty string if not found

    # Split the caption into lines that fit within the screen width
    lines = wrap_text(caption, font, font_scale, max_line_length)

    # Calculate the height of each line and the total height of the text block
    text_height = len(lines) * 30  # Each line's height is 30 pixels
    text_x = 50  # Left margin
    text_y = frame.shape[0] - 200  # Starting position for the first line

    # Find the width of the longest line
    max_line_width = 0
    for line in lines:
        (w, h), _ = cv2.getTextSize(line, font, font_scale, line_type)
        max_line_width = max(max_line_width, w)

    # Calculate the position of the background rectangle
    rect_start = (text_x - 10, text_y + (len(lines)+1) * 30)  # Top-left corner of the rectangle
    rect_end = (text_x + max_line_width + 10, text_y + 10 - 50)  # Bottom-right corner of the rectangle

    # Create a semi-transparent background rectangle
    overlay = frame.copy()
    cv2.rectangle(overlay, rect_start, rect_end, background_color, -1)
    # Blend the overlay with the original frame to add transparency
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    # Draw each line of text on top of the white background
    for i, line in enumerate(lines):
        # Calculate the correct Y position for each line of text
        current_text_y = text_y + i * 30  # Move down for each line
        cv2.putText(frame, line, (text_x, current_text_y), font, font_scale, font_color, line_type)

    # Now, print the `pos` and `theta` on the top-right corner
    pos_text = f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
    theta_text = f"Theta: {theta:.2f}"

    # Calculate the position for the top-right corner
    pos_width, _ = cv2.getTextSize(pos_text, font, font_scale, line_type)[0]
    theta_width, _ = cv2.getTextSize(theta_text, font, font_scale, line_type)[0]
    
    # Place the text in the top-right corner
    pos_x = frame.shape[1] - pos_width - 20  # 20 px margin from the right
    pos_y = 40  # Start a bit below the top

    # Draw the `pos` and `theta` text in the top-right corner
    cv2.putText(frame, pos_text, (pos_x, pos_y), font, font_scale, font_red_color, line_type)
    cv2.putText(frame, theta_text, (pos_x, pos_y + 30), font, font_scale, font_red_color, line_type)



def wrap_text(text, font, font_scale, max_line_length):
    """Wrap text to fit within the given max line length."""
    words = text.split(' ')
    lines = []
    current_line = ''

    for word in words:
        # Add the word to the current line and check if it exceeds the maximum length
        test_line = current_line + (word + ' ')
        (w, h), _ = cv2.getTextSize(test_line, font, font_scale, 2)
        
        if w < max_line_length:
            # If it fits, add the word to the current line
            current_line = test_line
        else:
            # If it doesn't fit, start a new line
            if current_line != '':
                lines.append(current_line.strip())
            current_line = word + ' '

    # Append the last line
    if current_line != '':
        lines.append(current_line.strip())

    return lines



def convert_bag_to_video(
    bag_path: str, storage_id: str, topic_name: str, output_path: str, codec: int, fps: float
):
    """Create a bagfile from a video."""
    # Force the .mp4 extension on file output name
    if pathlib.PurePath(output_path).suffix == '':
        output_path += '.mp4'
    image_reader = create_sequential_image_bag_reader(bag_path, storage_id, topic_name)
    cv_image, start_stamp = image_reader.next()
    video_writer = create_sequential_video_writer(output_path, codec, fps, cv_image, start_stamp)

    # Sort captions by time
    captions_sorted = sorted(captions, key=lambda x: x['time'])
    caption_index = 0
    current_caption = None
    caption_end_time = None  # Time when the current caption should stop being displayed

    while image_reader.has_next():
        cv_image, stamp = image_reader.next()

        # Print the current frame's timestamp
        print(f"Processing frame at time: {stamp}")

        # Check if we need to start displaying a new caption
        while caption_index < len(captions_sorted) and stamp >= (captions_sorted[caption_index]['time']-3.0):
            current_caption = captions_sorted[caption_index]
            caption_end_time = captions_sorted[caption_index]['time']  # Display for 2 seconds
            print(f"Starting caption: {current_caption} at time: {stamp}")
            caption_index += 1

        # Check if we need to stop displaying the current caption
        if current_caption and stamp >= caption_end_time:
            print(f"Stopping caption: {current_caption} at time: {stamp}")
            current_caption = None
            caption_end_time = None

        # Add the current caption to the frame if it exists
        if current_caption:
            add_caption_to_frame(cv_image, current_caption)
            print(f"Displaying caption: {current_caption} at time: {stamp}")

        video_writer.add_frame(cv_image, stamp)
    video_writer.close()

    print(
        f'Processed {video_writer.images_processed} messages and wrote '
        f'{video_writer.frames_written} frames. '
        f'{video_writer.images_skipped} messages were skipped',
        file=sys.stderr)
    print(f"Output video: {output_path}", file=sys.stderr)

def main(args):
    """
    Create a bagfile from a video.

    Wrapper of convert_bag_to_video(), that handles exceptions and prints errors instead.
    """
    codec = args.codec
    if args.codec_dialog:
        list_available_codecs()  # List available codecs if requested
        return  # Exit after listing codecs
    try:
        convert_bag_to_video(
            args.bagfile, args.storage_id, args.topic, args.output, codec, args.fps)
    except CommandInputError as e:
        print(e, file=sys.stderr)
    except Exception as e:
        print(f'Unexpected exception of type [{type(e)}]: {e}', file=sys.stderr)

