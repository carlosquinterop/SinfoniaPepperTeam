#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2 as cv2
from person import Person
from person import Less_Blurred
from edit_files import Group


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

    def indentify_person(self, cvWindow):
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        cap.release()
        personId = self.persons.identifyPerson(frame)

        if(cvWindow):
            pi = (self.persons.bb['left'], self.persons.bb['top'])
            pf = (self.persons.bb['left']+self.persons.bb['width'],
                  self.persons.bb['top']+self.persons.bb['height'])
            cv2.rectangle(frame, pi, pf, (0, 255, 0), 3)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, self.persons.name, pi, font,
                        1, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.imshow('image', frame)
            print("Press Enter to Exit")
            cv2.waitKey(0)
        return personId

    def detect_person(self, frame):
        people = self.persons.detectPerson(frame)
        videoSize = frame.shape[0]*frame.shape[1]
        max_prop = 0;
        for persons in people:
            prop = (persons['faceRectangle']['width']*persons['faceRectangle']['height'])*100/videoSize
            if prop > max_prop:
                max_prop = prop
                face_detected = persons
        if max_prop > 0:
            pi = (face_detected['faceRectangle']['left'],face_detected['faceRectangle']['top'])
            pf = (face_detected['faceRectangle']['left']+face_detected['faceRectangle']['width'],
                  face_detected['faceRectangle']['top']+face_detected['faceRectangle']['height'])
            cv2.rectangle(frame, pi, pf, (0, 255, 0), 3)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, str(max_prop)+'%', pi, font, 1, (255, 0, 0), 2, cv2.LINE_AA)
        _face_prop = 4
        print("Prop {}".format(max_prop))
        if max_prop > _face_prop:
            resp = True
        else:
            resp = False
        return resp, frame
    def get_persons_attributes(self):
        G = Group()
        for p in G.persons:
            print(p)
        return G.persons


# c = Characterization()
# c.indentify_person(True)
