#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cognitive_face as CF
import requests
import os
import sys
import json
from utils import Utils


class Azure:
    ALL_ATTRIBUTES = 'age,gender,headPose,smile,facialHair,glasses,emotion,makeup,hair,accessories'

    WORKING = -1
    INTERNET_ERROR = 1
    ANOTHER_ERROR = 2
    DETECT_ERROR = 3
    LIST_PERSON_ERROR = 4
    DELETE_PERSON = 5
    ADD_FACE = 6
    TRAIN_ERROR = 7
    NO_PERSON_IN_GROUP = 8
    VERIFY_RECOGNITION = 9
    VERIFY_DETECT = 10
    CREATE_PERSON = 11
    IDENTIFY = 12
    attributes = []
    def __init__(self):

        self.ROOT_PATH = os.path.dirname(sys.modules['__main__'].__file__)
        self.KEY, self.BASE_URL, self.largePersonGroupId = self.get_secrets(
            self.ROOT_PATH)
        CF.Key.set(self.KEY)
        CF.BaseUrl.set(self.BASE_URL)
        # self.delete_person_without_face()
        #self.personInfo = {}
        # self.get_all_names()
        self.codeError = -1
        self.utils = Utils()


    def get_secrets(self, ROOT_PATH):
        with open(ROOT_PATH+"/AZURE_SECRET_CREDENTIALS.json") as f:
            secretInfo = json.load(f)
            return secretInfo["KEY"], secretInfo["BASE_URL"], secretInfo["LARGE_PERSON_GROUP_ID"]

    def train(self):
        print("TRAINING")
        self.codeError = -1
        try:
            CF.large_person_group.train(
                large_person_group_id=self.largePersonGroupId)
            print("TRAINED!!!!")
        except requests.exceptions.ConnectionError:
            self.codeError = self.INTERNET_ERROR
        except:
            self.codeError = self.TRAIN_ERROR
        finally:
            if self.codeError != -1:
                print("falle en algo u.u")
                return False
        return True

    def get_face_rectangle(self, response):
        left = response['faceRectangle']['left']
        top = response['faceRectangle']['top']
        width = response['faceRectangle']['width']
        height = response['faceRectangle']['height']
        return str(left) + "," + str(top) + "," + str(width) + "," + str(height)

    def request_add_face(self, imageBytes, largePersonGroupId, personId, targetFace):
        url = 'largepersongroups/{}/persons/{}/persistedFaces'.format(
            largePersonGroupId, personId)
        headers, data, json = {
            'Content-Type': 'application/octet-stream'}, imageBytes, None
        headers['Ocp-Apim-Subscription-Key'] = self.KEY
        params = {
            'userData': '',
            'targetFace': targetFace,
        }
        response = requests.request(
            'POST',
            self.BASE_URL + url,
            params=params,
            data=data,
            json=json,
            headers=headers,
            verify=False)

        return response.json()

    def add_face(self, imageBytes, person_id):
        self.codeError = -1
        responseDetection = self.detect(imageBytes)
        if self.verify_detection(responseDetection):
            # self.attributes = self.extract_attributes(responseDetection)

            targetFace = self.get_face_rectangle(responseDetection[0])
            imgObject = Img(imageBytes)
            if person_id:
                try:
                    CF.large_person_group_person_face.add(imgObject, self.largePersonGroupId,
                                                          person_id,
                                                          target_face=targetFace)
                    print("Added face!!")
                except requests.exceptions.ConnectionError:
                    self.delete_person(person_id)
                    self.codeError = self.INTERNET_ERROR
                except Exception as e:
                    print("Error: {}".format(e))
                    self.delete_person(person_id)
                    self.codeError = self.ADD_FACE
                finally:
                    if self.codeError != -1:
                        print("wtf", self.codeError)
                        return False, self.codeError
                return True, self.WORKING
            else:
                return False, self.codeError
        else:
            self.codeError = self.VERIFY_DETECT
            return False, self.codeError

    def create_person(self, name):
        person_id = None
        try:
            person_id = CF.large_person_group_person.create(self.largePersonGroupId,
                                                            name)["personId"]
            print("The person {} was created with ID: {}".format(name, person_id))
        except requests.exceptions.ConnectionError:
            self.codeError = self.INTERNET_ERROR
        except:
            self.codeError = self.CREATE_PERSON
        return person_id, self.codeError

    def get_all_names(self):
        persons = []
        groupInfo = []
        try:
            persons = CF.large_person_group_person.list(
                self.largePersonGroupId)
            for person in persons:
                groupInfo.append(
                    {'name': person['name'], 'personId': person['personId']})
            print("group information was got ")
        except requests.exceptions.ConnectionError:
            self.codeError = self.INTERNET_ERROR
        except Exception as e:
            self.codeError = self.LIST_PERSON_ERROR
            print("[Errno {0}] {1}".format(e.errno, e.strerror))
        return groupInfo, self.codeError

    def detect(self, imageBytes, threshold=None, attributes=ALL_ATTRIBUTES):
        self.codeError = -1
        imgObject = Img(imageBytes)
        response = []
        try:
            response = CF.face.detect(imgObject, face_id=True, attributes=attributes)
            if response:
                print("Face detected ", len(response))
            else:
                print("No Face detected ")
        except requests.exceptions.ConnectionError:
            self.codeError = self.INTERNET_ERROR
        except:
            self.codeError = self.DETECT_ERROR
        return response

    def identify(self, imageBytes, threshold=None):
        self.codeError = -1
        responseDetection = self.detect(imageBytes)
        if len(responseDetection)>0:
            attributes = responseDetection[0]
            attributes["name"] = "desconocido"
            attributes["accuracy"] = 0
            attributes['id_azure'] = ""
            attributes['verify_recognition'] = False
            faceId = attributes["faceId"]
            try:
                responseIdentify = CF.face.identify([faceId],large_person_group_id=self.largePersonGroupId,threshold=threshold,)

            except requests.exceptions.ConnectionError:
                self.codeError = self.INTERNET_ERROR
            except:
                self.codeError = self.IDENTIFY
            finally:
                if self.codeError != -1:
                    return attributes, self.codeError

            if self.verify_recognition(responseIdentify):
                id_azure, accuracy = self.extract_recognition(responseIdentify)
                attributes["id_azure"] =  id_azure
                attributes["accuracy"] =  accuracy
                attributes['verify_recognition'] = True

            else:
                self.codeError = self.VERIFY_RECOGNITION
                if self.codeError != -1:
                    return attributes, self.codeError
        else:
            self.codeError = self.VERIFY_DETECT
            if self.codeError != -1:
                return attributes, self.codeError
        return attributes, self.WORKING

    def verify_detection(self, responseDetection):
        if len(responseDetection) > 0:
            return True
        return False

    def verify_recognition(self, responseIdentify):
        if len(responseIdentify[0]['candidates']) > 0:
            return True
        return False

    def extract_attributes(self, responseDetection):

        bb = responseDetection['faceRectangle']
        faceId = responseDetection['faceId']
        r = responseDetection['faceAttributes']
        age = r['age']
        gender = r['gender']
        smile = r['smile']
        mustache = r['facialHair']['moustache']
        beard = r['facialHair']['beard']
        sideburns = r['facialHair']['sideburns']

        if r['glasses'] == 'ReadingGlasses':
            glasses = 'eyeglasses'
        elif r['glasses'] == 'Sunglasses':
            glasses = 'sunglasses'
        elif r['glasses'] == 'NoGlasses':
            glasses = 'noglasses'
        else:
            glasses = ''
        pose = r['headPose']

        happiness = r['emotion']['happiness']
        sadness = r['emotion']['sadness']
        neutral = r['emotion']['neutral']
        surprise = r['emotion']['surprise']
        anger = r['emotion']['anger']

        bald = r['hair']['bald']
        if r['hair']['hairColor']:
            # asumo que el 0 es el mayor
            hairColor = r['hair']['hairColor'][0]['color']
        else:
            hairColor = ""
        eyeMakeup = r['makeup']['eyeMakeup']
        lipMakeup = r['makeup']['lipMakeup']

        mask, headWear = 0, 0
        for item in r['accessories']:
            if item['type'] == 'mask':
                mask = item['confidence']
            elif item['type'] == 'headWear':
                headWear = item['confidence']

        return {"faceId": faceId,
                "age": age,
                "gender": gender,
                "bb": bb,
                "smile": smile,
                "mustache": mustache,
                "beard": beard,
                "sideburns": sideburns,
                "glasses": glasses,
                "pose": pose,
                "bald": bald,
                "hairColor": hairColor,
                "eyeMakeup": eyeMakeup,
                "lipMakeup": lipMakeup,
                "headWear": headWear,
                "mask": mask,
                "happiness": happiness,
                "sadness": sadness,
                "neutral": neutral,
                "surprise": surprise,
                "anger": anger}

    def extract_recognition(self, responseIdentify):
        id_azure = responseIdentify[0]["candidates"][0]['personId']
        accuracy = responseIdentify[0]["candidates"][0]['confidence']
        #name = self.personInfo[id_azure]
        return id_azure, accuracy

    def delete_person(self, personId):
        try:
            CF.large_person_group_person.delete(
                self.largePersonGroupId, person_id=personId)
            print('Person Deleted')
        except requests.exceptions.ConnectionError:
            self.codeError = self.INTERNET_ERROR
            print('Internet Error In delete_person')
        except Exception as e:
            print("Error: {}".format(e))
            self.codeError = self.DELETE_PERSON
            print('Error In delete_person')

    def delete_person_without_face(self):
        try:
            responseInfo = CF.large_person_group_person.list(
                self.largePersonGroupId)
            print(responseInfo)
            for person in responseInfo:
                if not person['persistedFaceIds']:
                    self.delete_person(person['personId'])
        except requests.exceptions.ConnectionError:
            self.codeError = self.INTERNET_ERROR
        except:
            self.codeError = self.LIST_PERSON_ERROR

    def compare(self, present_faceId, new_image):
        self.codeError = -1
        responseDetection = self.detect_faces(new_image)
        print(responseDetection)
        attributes = None
        if self.verify_detection(responseDetection):
            attributes = self.extract_attributes(responseDetection)
            # todo detect no está en las excepciones
            new_faceId = responseDetection[0]['faceId']
            same_person = False

            try:
                compareResponse = CF.face.verify(present_faceId, new_faceId)
                if compareResponse['isIdentical']:
                    same_person = True
                # compareResponse["confidence"]
                print(compareResponse)

                return attributes, same_person
            except requests.exceptions.ConnectionError:
                self.codeError = self.INTERNET_ERROR
            except:
                print("fallando seguro porque pasó 10+")
            finally:
                return attributes, same_person
        else:
            # todo revisar este else
            return attributes, False


class Img:
    def __init__(self, image):
        self.image = image

    def read(self):
        return self.image
