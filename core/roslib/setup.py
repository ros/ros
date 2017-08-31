#!/usr/bin/env python

from setuptools import setup
import os

import xml.etree.ElementTree


class XmlNodes(object):
    """
    Simple API for xml.etree.ElementTree that collapses similar tags.
    Specially designed for our usage here.
    """

    def __init__(self, xml_elems):
        """Handle one list of xml elements"""
        self.xml_elems = xml_elems

    def __getattr__(self, item):
        children = XmlNodes([e for e in self.xml_elems if e.tag == item])
        if len(children.xml_elems) == 0:
            raise AttributeError
        else:
            return children

    def __call__(self, attrib_name=None):
        if attrib_name is None:  # return the text
            return ", ".join([e.text for e in self.xml_elems])
        else:  # return the values of that attribute
            return ", ".join([e.attrib.get(attrib_name) for e in self.xml_elems])

    def filter(self, attrib_name, attrib_value):
        """special function to remove nodes we are not interested in"""
        return XmlNodes([e for e in self.xml_elems if e.attrib.get(attrib_name) == attrib_value])


class PackageXml(XmlNodes):
    """Extracts data from a package.xml file"""
    def __init__(self, package_xml_path='package.xml'):
        tree = xml.etree.ElementTree.parse(package_xml_path)
        root = tree.getroot()
        assert root.tag == 'package'
        # TODO : check package format version
        super(PackageXml, self).__init__([c for c in root])  # here we get all children already


ros_package = PackageXml(os.path.join(os.path.dirname(__file__), 'package.xml'))

setup(
    # based on package.xml
    name='ros_' + ros_package.name(),
    version=ros_package.version(),
    description=ros_package.description(),
    url=ros_package.url.filter('type', 'repository')(),
    author=ros_package.author(),
    maintainer_email=ros_package.maintainer('email'),
    license=ros_package.license(),
    # based on directory hierarchy
    packages=['ros', 'roslib'],
    package_dir={'': 'src'},
    install_requires=['catkin', 'rospkg'],
)
