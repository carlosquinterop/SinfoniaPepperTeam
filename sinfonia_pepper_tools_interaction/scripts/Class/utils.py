



class Utils:
    def setProps(self, people):
        props = [] 
        prop = 0
        for face_detected in people:
            pi = (face_detected['faceRectangle']['left'],face_detected['faceRectangle']['top'])
            pf = (face_detected['faceRectangle']['left']+face_detected['faceRectangle']['width'],
                  face_detected['faceRectangle']['top']+face_detected['faceRectangle']['height'])
            prop = (face_detected['faceRectangle']['width']*face_detected['faceRectangle']['height'])
            props.append({"pi":pi,"pf":pf,"prop":prop})#guarda el calculo de las proporciones en cada ciclo  
        return props