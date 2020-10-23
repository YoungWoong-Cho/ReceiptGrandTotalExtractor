# Made by YoungWoong Cho
# MEng major, CS minor in The Cooper Union
# https://github.com/YoungWoong-Cho

####################################################################################################################
## Note: On top of importing pytesseract, this program requires tesseract-OCR installation which is impossible in ##
## Google App Engine Standard environment. Therefore it was required to use Google Cloud Run which enables a      ##
## containerized application with Dockerfile. Please refer to the following link:                                 ##
## https://stackoverflow.com/questions/57869385/can-not-make-tesseract-work-in-google-app-engine-with-python3     ##
## > gcloud builds submit --tag gcr.io/youngwoong-cho/ocr                                                         ##
## > gcloud beta run deploy --image gcr.io/youngwoong-cho/ocr --platform managed                                  ##
## > youngwoong-cho-receipt-extractor                                                                             ##
####################################################################################################################

from flask import Flask, render_template, request
import pytesseract
import numpy as np
import cv2 as cv2
import os

def get_string(img):

    # Apply grayscale    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply dilation and erosion to remove some noise
    kernel = np.ones((1, 1), np.uint8)
    img = cv2.dilate(img, kernel, iterations=2)
    img = cv2.erode(img, kernel, iterations=1)
    img = cv2.GaussianBlur(img, (5,5), 0)

    #  Apply threshold to get image with only black and white
    img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 9)

    # Recognize text with tesseract for python
    text = pytesseract.image_to_string(img)
    
    # Extract grand total from the string
    text = text.replace(" ", "").replace(" ", "").replace(" ","")
    grand_total_index = text.find("Total")
    dollar_sign = text.find("$", grand_total_index)
    dot = text.find(".", dollar_sign)
    grand_total = float(text[dollar_sign+1:dot+3])
    
    return ("Grand Total: $%0.2f" %(grand_total))

app = Flask(__name__)

@app.route('/')
def index():
    return render_template("index.html", text = "")

@app.route("/",methods=['POST'])
def readText():
    try:
        if request.method == "POST":
            if request.form.get("button", False) == 'Extract':
                if request.files['file']: # if some file is chosen
                    #read image file string data
                    filestr = request.files['file'].read()
                    #convert string data to numpy array
                    npimg = np.fromstring(filestr, np.uint8)
                    img = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
                    mytext=get_string(img)
                else:
                    mytext = "Please choose file"
            elif request.form.get("button", False) == 'Try sample receipt':
                img = cv2.imread('static/receipt.jpg')
                mytext=get_string(img)
        else:
            mytext="not posted"
    except:
        mytext = "There was a problem processing. Please retry."
        
    return render_template("index.html", text=mytext)

if __name__ == "__main__":
    app.run(debug=True,host='0.0.0.0',port=int(os.environ.get('PORT', 8080)))