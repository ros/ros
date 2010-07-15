
# <WikiCodeTag(WRITE_PY)>
import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('test.bag', 'w')

str = String()
str.data = 'foo'

i = Int32()
i.data = 42

bag.write('chatter', str);
bag.write('numbers', i);

bag.close();
# </WikiCodeTag(WRITE_PY)>
