from email.mime import image
import cv2
import numpy as np
import easyocr
import time
from imutils import rotate_bound
from statistics import mode
from defisheye import Defisheye
from imutils.perspective import four_point_transform
DEBUG = True

class displayDetection:
    
    def __init__(self,period):

        # period eh o intervalo de tempo para medir (limite da competicao ou oq vcs decidirem)
        # parameter eh a informacao que vcs querem, 1 = gas, 2 = zero adj e 3 = array com os dois

        # ver imagem rotacionada no teste, se ela ficar virada em 180 graus, vamos inverter
        # para o teste com a camera do drone, ela rotacionou certo 

        self.cap = cv2.VideoCapture(0)
        self.degrees = 0
        self.squares = []
        self.period = period
        self.gas_percentual_list1 = []
        self.gas_percentual_list2=[]
        self.zero_adjustment_list = []
        self.reader = easyocr.Reader(['pt'])
        self.gas_percentual_image = []
        self.gas_percentual = []
        self.result = []
        self.zero_adjustment = []
    

    
    def find_squares(self, contours):

        #Essa função acha os quadrados que podem ou não ser quadrados

        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspectRatio = float(w) / h


                if aspectRatio >= 0.95 and aspectRatio < 1.1 and cv2.contourArea(contour) > 2000 and cv2.contourArea(contour) < 30000:

                    self.reshape = True
                    self.reshaped_image = four_point_transform(self.image, approx.reshape(4,2)) 

                    if DEBUG:
                        cv2.imshow("reshape", self.reshaped_image)
                    
                    #self.squares.append(contour)
                    if DEBUG:
                        cv2.drawContours(self.image, [approx], 0, (255, 0, 0), 4)
                    #cv2.rectangle(self.image,(x,y),(x+y, y+h), (0,255,0),2)

    def find_reshaped_square(self):

        #Essa função pega o quadrado achado pela find_squares e aplica uma transformação de 4 pontos para 
        #deixar o quadrado bonitinho. Além disso, ela verifica se realmente é um display utilizando a função FindCircles

        self.squares= []

        gray = cv2.cvtColor(self.reshaped_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        ret, thresh = cv2.threshold(gray, 120, 255, cv2.CHAIN_APPROX_NONE)
        #thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 5, 2)
        # cv2.imshow("thresh2", thresh)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspectRatio = float(w) / h


                if aspectRatio >= 0.95 and aspectRatio < 1.1 and cv2.contourArea(contour) > 2000:
                    
                    circles = self.findCircles()

                    if circles:
                        self.squares.append(contour)
                        # cv2.imshow("reshape", self.reshaped_image)
                        #cv2.drawContours(self.reshaped_image, [approx], 0, (255, 0, 0), 4)

                


    def crop_image(self):

        # Essa função é extremamente importante, ela recorta todos as regiões de interesse da imagem, que serão passadas para o 
        # algoritimo de OCR, essas regiões de interesse correspondem aos dois algarismos do percentual de gás,
        # ao algarismo da unidade do ajuste de zero, à região do ajuste de zero onde pode haver um 1 ou não, e ao
        # local do ajuste de zero onde pode haver um sinal de negativo ou não


        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(self.reshaped_image, kernel)
        erode = cv2.erode(dilated, (5,5))
            #blurred = cv2.GaussianBlur(dilated, (3, 3), 0)
            #canny = cv2.Canny(self.image,100,200)
            #cv2.imshow("canny", canny)
        

        x = np.where(self.reshaped_image > 0)[0]
        y = np.where(self.reshaped_image > 0)[1]
        x1 = np.min(x)
        x2 = np.max(x)
        y1 = np.min(y)
        y2 = np.max(y)


        image_copy = erode.copy()
        cropped_image = image_copy[x1:x2, y1:y2]
                                                                    
                                                                    ##ATENÇÂO## 

        # Abaixo é onde são recortadas todas as zonas de interesse, os recortes foram feitos a partir de parâmetros obtidos experimentalmente
        # é possível que mudando-se condições como a resolução da câmera, esses parâmetros divirjam um pouco,
        # o que pode causar problemas na hora da detecção com o OCR, então cuidado com isso. 


        #crop the top side of the image (gas percentual)
        cropped_image1 = cropped_image[round(cropped_image.shape[0]*0.02):round(cropped_image.shape[0]*0.57), 0:round(cropped_image.shape[1]*0.65)]
        #cv2.imshow("crop", cropped_image1)
        #crop the bottom side of the image (gas percentual)
        cropped_image2 = cropped_image[round(cropped_image.shape[0]/2):round(cropped_image.shape[0]), round(cropped_image.shape[1]*0.06):round(cropped_image.shape[1]*0.66)]

        #crop the first character of the gas percentual number
        cropped_image3 = cropped_image1[0:cropped_image1.shape[0], 0:round(cropped_image1.shape[1]*0.54)]

        #crop the second character of the gas percentual number
        cropped_image4 = cropped_image1[0:cropped_image1.shape[0], round(cropped_image1.shape[1]*0.52):round(cropped_image1.shape[1]*0.99)]
        
        #cv2.imshow("crop2", cropped_image4)
        #crop the zero adjustment number
        cropped_image5 = cropped_image2[0:cropped_image2.shape[0], round(cropped_image2.shape[1]*0.49):round(cropped_image2.shape[1])]
        
    
        #crop the image that may contain 1 or not
        cropped_image6 = cropped_image2[0:round(cropped_image2.shape[0]*0.90), round(cropped_image2.shape[1]*0.40):round(cropped_image2.shape[1]*0.51)]

        #crop the image that may contain "-" or not
        cropped_image7 = cropped_image2[round(cropped_image2.shape[0]*0.43):round(cropped_image2.shape[0]*0.57), round(cropped_image2.shape[1]*0.055):round(cropped_image2.shape[1]*0.36)]

        if DEBUG:

            cv2.imshow("rotacionada", self.reshaped_image)
            cv2.imshow("crop1", cropped_image3)
            cv2.imshow("crop2", cropped_image4)
            cv2.imshow("crop3", cropped_image5)
            cv2.imshow("one", cropped_image6)
            cv2.imshow("minus", cropped_image7)

        #append the numbers images in their lists

        self.gas_percentual_image = []


        #Após os cortes serem realizados, guarda as imagens das regiões de interesse nas respectivas variaveis

        self.gas_percentual_image.append(cropped_image3)
        self.gas_percentual_image.append(cropped_image4)
        self.zero_adjustment_image = cropped_image5
        self.one_image = cropped_image6
        self.minus_image = cropped_image7

        
    #Optical Character Recognition
    def OCR(self, image):

        # Esta função aplica o algoritimo de Optical Character Recognition para as imagens, ela retorna o caractere reconhecido
        # com sua respectiva acurácia, neste caso, retornamos o valor apenas se a acurácia for maior que 0.8 e se encontrar um caractere
        # que possa ser transformado em um inteiro (para evitar detecçoes de letras e simbolos)

        result = self.reader.readtext(image)
        if DEBUG:

            print(result)
        

        if result:

            content = result[0][1]
            accuracy = result[0][2]

            if accuracy > 0.80 and self.checkInt(content):
                
                return [content, accuracy]

            else: 

                return [False, False]

        else:

            return [False, False]


    def checkInt(self,str):

        #checa se uma string pode ser transformada em um numero inteiro ou não

        try:
            int(str)
            return True
        except ValueError:
            return False

    #check if there is something in a image
    def findCircles(self):

        # Essa função recorta a imagem nos locais supostos de terem o simbolo "%", se ela encontrar o simbolo tanto 
        # em cima quanto embaixo, significa que dentro do quadrado realmente tem um display, se ela não encontrar logo de cara,
        # ela rotaciona a imagem transformada em 90 graus até que os dois simbolos de "%" sejam encontrados (se existirem)

        finded = False
        i = 0

        (h, w) = self.reshaped_image.shape[:2]
        (cX, cY) = (w // 2, h // 2)

        
        rotated = self.reshaped_image

        while (not finded) and i < 4:

            top = False
            bot = False
            circlesTop = None
            circlesBot = None
            #rotated = self.reshaped_image
            gray = cv2.cvtColor(self.reshaped_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.blur(gray, (3,3))
            ret, thresh = cv2.threshold(blurred, 110, 255, cv2.CHAIN_APPROX_TC89_L1)
            contours,h = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)

            canny = cv2.Canny(self.reshaped_image, 100, 200)
            topPiece = thresh[0:round(thresh.shape[0]/2), round(thresh.shape[1]*0.5):round(thresh.shape[1])]
            bottomPiece = thresh[round(thresh.shape[0]/2):round(thresh.shape[0]), round(thresh.shape[1]*0.5):round(thresh.shape[1])]

            contoursTop,h = cv2.findContours(topPiece,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)
            contoursBot,h = cv2.findContours(bottomPiece,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)

            if DEBUG:

                cv2.imshow("percentage1", topPiece)
                cv2.imshow("percentage2", bottomPiece)
            
            
            for cnt in contoursTop:
                #acha circulos em cima
                approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
        
                if len(approx)==8:

                    k = cv2.isContourConvex(approx)
                    if k:

                        if DEBUG:

                            cv2.drawContours(topPiece, [cnt], 0, (220, 152, 91), -1)

                        top = True
                                     
            for cnt in contoursBot:

                #acha circulos embaixo
                approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
        
                if len(approx)==8:

                    k = cv2.isContourConvex(approx)

                    if k:

                        if DEBUG:
                            cv2.drawContours(bottomPiece, [cnt], 0, (220, 152, 91), -1)

                        bot = True

            if bot and top:

                finded = True
            
            else:
                
                #rotaciona a imagem
                (h, w) = rotated.shape[:2]
                (cX, cY) = (w // 2, h // 2)

                M = cv2.getRotationMatrix2D((cX, cY), 90, 1.0)
                self.reshaped_image = cv2.warpAffine(self.reshaped_image, M, (w, h))
                # rotated = rotate_bound(self.reshaped_image,-90)
                # self.reshaped_image=rotated 
                #print("cheguei")
                #cv2.imshow("Rotated by 180 Degrees", rotated)
                
                i = i + 1
                     
            if cv2.waitKey(5) & 0xFF == 27:
                break

        if finded:


            # self.degrees = i*90
            
            return True

        else:

            return False


        
    def isEmpty(self, image, tolerance):

        # Esta função é usada para verificar se há algo na imagem ou não
        # Neste codigo ela é aplicada principalmente para os recortes das imagens nas quais contem
        # um sinal de menos ou não, e também para saber se no valor do ajuste de zero possui um 1 ou não

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(gray, 120, 255, cv2.CHAIN_APPROX_NONE)
        # if(image.shape == self.one_image.shape):
        #     cv2.imshow("thresh funcao", thresh)

        total = thresh.size
        zero = total - np.count_nonzero(thresh)

        # a variavel ratio representa a razão entre o numero de pontos brancos e pretos (os numeros sao brancos)
        # portanto, se esta razão for maior que a tolerancia definida, há algo na imagem

        ratio = zero/total     
    
        if ratio > tolerance:
            return True

        else:
            return False


    def detection_loop(self):

        #loop principal de detecção

        i = 0
        # self.reshape = False
        start = time.time()
        difTime = 0
        while self.cap.isOpened() and difTime < self.period:

            self.reshape = False

            self.squares = []
            
            success, self.image = self.cap.read()

            #Define os parametros da camera para a aplicação do Defisheye
            dtype = 'linear'
            format = 'fullframe'
            fov = 140
            pfov = 90

            
            img_out = f"./images/out/TESTE_{dtype}_{format}_{pfov}_{fov}.jpg"
            

            self.image = Defisheye(self.image, dtype=dtype, format=format, fov=fov, pfov=pfov)
            # obj.convert(img_out)
            
            self.image = self.image.convert(img_out)

            # Aplica algumas transformações na imagem original para encontrar o quadrado


            # self.image = cv2.resize(self.image, (1280,720))
            kernel = np.ones((3, 3), np.uint8)
            dilated = cv2.dilate(self.image, kernel)
            #erode = cv2.erode(self.image, kernel)
            #blurred = cv2.GaussianBlur(dilated, (3, 3), 0)
            #canny = cv2.Canny(self.image,100,200)
            #cv2.imshow("canny", canny)
            gray = cv2.cvtColor(dilated, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            


                                            ##ATENÇÂO##
            # o threshold é um parãmetro meio sensivel então era bom ficar atento no comportamento dele na hora de realizar os testes
            # tentamos aplicar o thresh hold mas apesar de ser mais bonitinho, n tava dando mt bom, se quiserem dar uma olhada está comentado abaixo


            thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 3, 2)
            #ret, thresh = cv2.threshold(gray, 110, 255, cv2.CHAIN_APPROX_NONE)

            if DEBUG:
                cv2.imshow("thresh", thresh)


            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            self.find_squares(contours)
            

            if self.reshape:
                self.find_reshaped_square()
                


            if self.squares:

                # se chegamos até aqui, significa que o codigo ja passou pela verificação do quadrado e também dos circulos que representam os 
                # sinais de porcentagem, portanto já podemos dizer que encontramos o display

                if(i==0):

                    print("DETECTADO")


                #sorted_squares = sorted(self.squares, key = cv2.contourArea, reverse = True )
                #mask = np.zeros(self.image.shape, np.uint8)
                #cv2.drawContours(mask, [sorted_squares[0]], 0, (0,0, 255), -1, )
                #cv2.drawContours(self.image, [sorted_squares[0]], 0, (0,0, 255), -1, )
                #cv2.imshow("mask", mask)

                self.crop_image()

                
                
                #apply the OCR algorithm over the numbers images
                
                self.gas_percentual = [0,0]
                self.zero_adjustment = 0

                self.gas_percentual = [self.OCR(self.gas_percentual_image[0])[0], self.OCR(self.gas_percentual_image[1])[0]]                
                self.zero_adjustment = self.OCR(self.zero_adjustment_image)[0]

                one = self.isEmpty(self.one_image, 0.35)
                minus = self.isEmpty(self.minus_image, 0.35)


                if self.gas_percentual[0]:
                    self.gas_percentual_list1.append(self.gas_percentual[0])
                
                else:

                    # Esta parte do codigo é para tentar evitar erros. Como o numero mais dificil pra IA reconhecer é o 1,
                    # quando nada é reconhecido, checamos se há algo no espaço correspondente.
                    # Se houver algo, assumimos que ali existe um 1 que o codigo nao conseguiu reconhecer

                    oneGasPercentual0 = self.isEmpty(self.gas_percentual_image[0], 0.35)
                    if len(self.squares)==(i+1) and oneGasPercentual0 :
                        self.gas_percentual_list1.append(1)


                if self.gas_percentual[1]:
                                            
                    
                    self.gas_percentual_list2.append(self.gas_percentual[1])
                   

                else:

                    oneGasPercentual1 = self.isEmpty(self.gas_percentual_image[1], 0.35)
                    if len(self.squares)==(i+1) and oneGasPercentual1:
                        self.gas_percentual_list2.append(1)
                        
            

                if self.zero_adjustment:

                # A partir dos resultados do OCR das imagens correspondentes as posiçoes do 1 e do sinal de negativo,
                # adicionamos estas condiçoes no valor final caso existam

                    if one:
                        self.zero_adjustment = int(self.zero_adjustment) + 10

                    if minus:
                        self.zero_adjustment = int(self.zero_adjustment)*-1

                    self.zero_adjustment_list.append(self.zero_adjustment)
                    
                    

                else: 
                    oneZeroAdj = self.isEmpty(self.zero_adjustment_image, 0.35)
                    if len(self.squares)==(i+1) and oneZeroAdj:

                        self.zero_adjustment =1

                        if one:
                            self.zero_adjustment = int(self.zero_adjustment) + 10

                        if minus:
                            self.zero_adjustment = int(self.zero_adjustment)*-1

                        self.zero_adjustment_list.append(self.zero_adjustment)
                i = i+1

            end = time.time()
            difTime = end - start

            #cv2.imshow("image", self.image)
            #if cv2.waitKey(5) & 0xFF == 27:
            #    break
            # print("len1:",len(self.gas_percentual_list1))   
            # print("len2:",len(self.gas_percentual_list2)) 
            # print("len3: ", len(self.zero_adjustment_list))

        if(len(self.gas_percentual_list1)!=0 and len(self.gas_percentual_list2)!=0):

            self.gas_percentual[0]=mode(self.gas_percentual_list1)
            self.gas_percentual[1]=mode(self.gas_percentual_list2)

            self.gasPercentual = int(str(self.gas_percentual[0]) + str(self.gas_percentual[1]))

        else:

            self.gasPercentual = 0

        if(len(self.zero_adjustment_list)!=0):

            self.zero_adjustment=int(mode(self.zero_adjustment_list))
        
        else: 

            self.zero_adjustment = 100


        return self.gasPercentual, self.zero_adjustment


            
    def main_interface(self):
        
        result = self.detection_loop()
        return result

#detection = displayDetection(20)
#result = detection.main_interface()
#print(result)


