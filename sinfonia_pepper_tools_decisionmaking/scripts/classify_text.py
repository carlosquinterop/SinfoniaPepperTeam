#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sinfonia_pepper_tools_decisionmaking.srv import *
import rospy
import six
import getpass
from google.cloud import language
from google.cloud.language import enums
from google.cloud.language import types
from google.cloud import language_v1
import os
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
username = getpass.getuser()

os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = './src/sinfonia_pepper_tools_decisionmaking/include/decision_making/claves.json'
def callback_classifytext(req):
    print ("Returning [%s]"%(req.text))
    text = req.text
    client = language_v1.LanguageServiceClient()
    if isinstance(text, six.binary_type):
        text = text.decode('utf-8')
    document = types.Document(
        content=text,
        type=enums.Document.Type.PLAIN_TEXT)
    response = client.analyze_sentiment(document)
    sentiment = response.document_sentiment
    entities = client.analyze_entities(document).entities
    Nombre=''
    Orden = []
    Afirmacion=False
    Negacion=False
    for entity in entities:
        entity_type = enums.Entity.Type(entity.type)
        if(entity_type.name=='PERSON'):
                Nombre=entity.name
                print(entity.name)
        elif(entity_type.name=='CONSUMER_GOOD'):
                Orden.append(entity.name)
    if sentiment.score<=-0.1 and Nombre=='' and Orden==[] :
        Afirmacion=False
        Negacion=True
    elif sentiment.score>-0.1 and Nombre==''and Orden==[]:
        Afirmacion=True
        Negacion=False
    return classify_textResponse(Afirmacion,Negacion,Nombre,Orden)


def callback_options():
    rospy.init_node('sIA_classify_text')
    s = rospy.Service('srv_classify_text', classify_text, callback_classifytext)
    print ("Ready")
    rospy.spin()


if __name__ == "__main__":
    callback_options()
