#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#!/usr/bin/env python
# license removed for brevity

"""
//======================================================================//
//  This software is free: you can redistribute it and/or modify        //
//  it under the terms of the GNU General Public License Version 3,     //
//  as published by the Free Software Foundation.                       //
//  This software is distributed in the hope that it will be useful,    //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of      //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE..  See the      //
//  GNU General Public License for more details.                        //
//  You should have received a copy of the GNU General Public License   //
//  Version 3 in the file COPYING that came with this distribution.     //
//  If not, see <http://www.gnu.org/licenses/>                          //
//======================================================================//
//                                                                      //
//      Copyright (c) 2019 SinfonIA Pepper RoboCup Team                 //
//      Sinfonia - Colombia                                             //
//      https://sinfoniateam.github.io/sinfonia/index.html              //
//                                                                      //
//======================================================================//
"""

import cv2 as cv2
try:
    from person import Person
    from person import Less_Blurred
    from edit_files import Group
except:
    from Class.person import Person
    from Class.person import Less_Blurred
    from Class.edit_files import Group

#import unicodedata

class Characterization:
    def __init__(self):
        self.persons = Person()
        self.blurry = Less_Blurred()

    def get_persons(self):
        personsList = self.persons.persons_in_group()
        for p in personsList:
            print(p)
        return personsList

    def add_person(self, name, images):
        self.blurry.sort_less_blurred(images)
        personId = self.persons.enrol(name, self.blurry.frames)
        return personId, self.persons

    def delete_person(self, name):
        self.persons.delete_person_by_name(name)

    def indentify_person(self, frame):
        people = self.persons.identifyPerson(frame)
        return people

    def detect_person(self, frame):
        people = self.persons.detectPerson(frame)
        return people

    def get_persons_attributes(self):
        G = Group()
        for p in G.persons:
            print(p)
        return G.persons


# c = Characterization()
# c.indentify_person(True)
