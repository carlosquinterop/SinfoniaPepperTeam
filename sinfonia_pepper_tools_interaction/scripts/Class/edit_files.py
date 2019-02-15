#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from io import open
import pickle

class PersonFiles:

    # Constructor de clase
    def __init__(self, name, personId, hairColor=" ", glasses=" ", gender=" ", age=0):
        self.name = name
        self.personId = personId
        self.hairColor = hairColor
        self.glasses = glasses
        self.gender = gender
        self.age = age
        print('The person {} has been saved in file personsGroup.pckl'.format(self.name))

    def __str__(self):
        return '{} has {} hair, {}, is approximately {} years old and is {}'.format(
                self.name, self.hairColor, self.glasses, self.age, self.gender)

class Group:

    persons = []
    temp = []

    # Constructor de clase
    def __init__(self):
        self.load()

    def add(self,p):
        self.persons.append(p)
        self.save()

    def show(self):
        if len(self.persons) == 0:
            print("The group is empty in edit_files")
            return
        
        for p in self.persons:
            print(p)

    def load(self):
        file = open('personsGroup.pckl', 'ab+')
        file.seek(0)
        try:
            self.persons = pickle.load(file)
        except:
            print("The File is Empty in edit_files")
        finally:
            file.close()
            print("{} persons loaded in edit_files".format(len(self.persons)))
    
    def delete(self,name):
        self.load()
        temp = []
        flag = False
        if len(self.persons) == 0:
            print("The group is empty in edit_files")
            return
        for person in self.persons:
            if person.name != name:
                temp.append(person)
            else:
                flag = True
                print("{} deleted in editfiles".format(name))
        if not flag:
            print("{} is not in group in editfiles".format(name))
        self.persons = temp;
        self.save()
        
    def save(self):
        file = open('personsGroup.pckl', 'wb')
        pickle.dump(self.persons, file)
        file.close()
