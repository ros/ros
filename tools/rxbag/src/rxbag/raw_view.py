# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Defines a raw view: a TopicMessageView that displays the message contents in a tree.
"""

PKG = 'rxbag'
import roslib; roslib.load_manifest(PKG)

import codecs
import math

import wx

import rospy

import bag_helper
from plugin.topic_message_view import TopicMessageView

class RawView(TopicMessageView):
    name = 'Raw'
    
    def __init__(self, timeline, parent):
        TopicMessageView.__init__(self, timeline, parent)
        
        self.msg_tree = MessageTree(self.parent)
        #self.msg_tree.Position = (0, 0)

        self.msg_displayed = None
        self.msg_incoming  = None
        
        self.parent.Bind(wx.EVT_PAINT, self.on_paint)
        self.parent.Bind(wx.EVT_SIZE,  self.on_size)

    ## MessageView implementation

    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)

        topic, msg, t = msg_details

        if t is None:
            self.message_cleared()
        else:
            self.msg_incoming = msg
            wx.CallAfter(self.parent.Refresh)

    def message_cleared(self):
        TopicMessageView.message_cleared(self)

        self.clear()

    ## Events

    def on_paint(self, dc):
        if self.msg_incoming != self.msg_displayed:
            self.msg_tree.set_message(self.msg_incoming)
            self.msg_displayed = self.msg_incoming

    def on_size(self, event):
        self.msg_tree.Size = self.parent.ClientSize

    ##

    def clear(self):
        self.msg_incoming = None
        wx.CallAfter(self.parent.Refresh)

class MessageTree(wx.TreeCtrl):
    def __init__(self, parent):
        wx.TreeCtrl.__init__(self, parent, style=wx.TR_DEFAULT_STYLE | wx.TR_HIDE_ROOT | wx.TR_MULTIPLE)

        self._font = wx.Font(9, wx.FONTFAMILY_MODERN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)

        self._msg = None

        self._expanded_paths = None

        self.Bind(wx.EVT_KEY_UP, self.on_key_up)

    @property
    def msg(self):
        return self._msg
    
    def set_message(self, msg):
        # Remember whether items were expanded or not before deleting
        if self._msg:
            for item in self.get_all_items():
                path = self.get_item_path(item)
                if self.IsExpanded(item):
                    self._expanded_paths.add(path)
                elif path in self._expanded_paths:
                    self._expanded_paths.remove(path)

            self.DeleteAllItems()

        if msg:
            # Populate the tree
            self._add_msg_object(None, '', 'msg', msg, msg._type)
            
            if self._expanded_paths is None:
                # First message: expand top-level items (that aren't the header)
                self._expanded_paths = set()
                for item in self.get_all_items():
                    path = self.get_item_path(item)
                    if '.' not in path and path != 'header':
                        self.Expand(item)
            else:
                # Expand those that were previously expanded, and collapse any paths that we've seen for the first time
                for item in self.get_all_items():
                    path = self.get_item_path(item)
                    if path in self._expanded_paths:
                        self.Expand(item)
                    else:
                        self.Collapse(item)

        self._msg = msg

        self.Refresh()

    def on_key_up(self, event):
        key, ctrl = event.KeyCode, event.ControlDown()

        if ctrl:
            if key == ord('C') or key == ord('c'):
                # Ctrl-C: copy text from selected items to clipboard
                self._copy_text_to_clipboard()
            elif key == ord('A') or key == ord('a'):
                # Ctrl-A: select all
                self._select_all()

    def _select_all(self):
        first_selected = self.GetFirstVisibleItem()
        for i in self.get_all_items():
            if not self.IsSelected(i):
                self.SelectItem(i, True)
                
        if first_selected is not None:
            self.ScrollTo(first_selected)

    def _copy_text_to_clipboard(self):
        # Get the indented text for all selected items
        
        def get_distance(item, ancestor, distance=0):
            parent = self.GetItemParent(item)
            if parent == ancestor:
                return distance
            else:
                return get_distance(parent, ancestor, distance + 1)
        
        root = self.GetRootItem()
        text = '\n'.join([('\t' * get_distance(i, root)) + self.GetItemText(i) for i in self.GetSelections()])

        # Copy the text to the clipboard
        if wx.TheClipboard.Open():
            try:
                wx.TheClipboard.SetData(wx.TextDataObject(text))
            finally:
                wx.TheClipboard.Close()

    def get_item_path(self, item):
        return self.GetItemPyData(item)[0]

    def get_all_items(self):
        items = []
        self.traverse(self.RootItem, items.append)
        return items

    def traverse(self, root, function):
        if self.ItemHasChildren(root):
            first_child = self.GetFirstChild(root)[0]
            function(first_child)
            self.traverse(first_child, function)

        child = self.GetNextSibling(root)
        if child:
            function(child)
            self.traverse(child, function)

    def _add_msg_object(self, parent, path, name, obj, obj_type):
        label = name
        
        if hasattr(obj, '__slots__'):
            subobjs = [(slot, getattr(obj, slot)) for slot in obj.__slots__]
        elif type(obj) in [list, tuple]:
            len_obj = len(obj)
            if len_obj == 0:
                subobjs = []
            else:
                w = int(math.ceil(math.log10(len_obj)))
                subobjs = [('[%*d]' % (w, i), subobj) for (i, subobj) in enumerate(obj)]
        else:
            subobjs = []
        
        if type(obj) in [int, long, float]:
            if type(obj) == float:
                obj_repr = '%.6f' % obj
            else:
                obj_repr = str(obj)

            if obj_repr[0] == '-':
                label += ': %s' % obj_repr
            else:
                label += ':  %s' % obj_repr

        elif type(obj) in [str, bool, int, long, float, complex, rospy.Time]:
            # Ignore any binary data
            obj_repr = codecs.utf_8_decode(str(obj), 'ignore')[0]
            
            # Truncate long representations
            if len(obj_repr) >= 50:
                obj_repr = obj_repr[:50] + '...'
            
            label += ': ' + obj_repr

        if parent is None:
            item = self.AddRoot(label)
        else:
            item = self.AppendItem(parent, label)

        self.SetItemFont(item, self._font)
        self.SetItemPyData(item, (path, obj_type))

        for subobj_name, subobj in subobjs:
            if subobj is None:
                continue
            
            if path == '':
                subpath = subobj_name                       # root field
            elif subobj_name.startswith('['):
                subpath = '%s%s' % (path, subobj_name)      # list, dict, or tuple
            else:
                subpath = '%s.%s' % (path, subobj_name)     # attribute (prefix with '.')

            if hasattr(subobj, '_type'):
                subobj_type = subobj._type
            else:
                subobj_type = type(subobj).__name__

            self._add_msg_object(item, subpath, subobj_name, subobj, subobj_type)
