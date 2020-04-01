"""
The module to perform supervised classification
"""

#importing libraries
import sklearn
from sklearn import svm
from sklearn.externals import joblib

import numpy as np

import sys


class Feature_Classifier:
    '''
    The class to classify patterns with engineered features
    '''

    #constructor
    def __init__(self):
         self.SVMClassifier = 0
    
    def SupportVectorMachine(self, featureVector):
        '''
        Method to classify using Support Vector Machine
        '''
        
        #initilizing the result list
        classificationResult = []
        
        #classification by finding scores of every class
        #classScores = self.SVMClassifier.decision_function(featureVector)
        classScores = self.SVMClassifier.predict_proba(featureVector)
        
        #saving the class scores
        classificationResult.append(classScores)
        
        return classificationResult
    
    def LoadLearningMemory(self, classifierType, classifier_source_folder, classifier_folder_name, DST_Enabled):
        '''
        Method to load the stored training data
        '''
        
        #deciding on which classifier to load
        if classifierType == "SupportVectorMachine":
            if (DST_Enabled):
                fileAddress = classifier_source_folder + '/' + classifier_folder_name + '/Dempster-Shafer/SVM.pkl'
            else:
                fileAddress = classifier_source_folder + '/' + classifier_folder_name + '/SVM.pkl'
            SVMFile = open(fileAddress, "rb")
            self.SVMClassifier = joblib.load(SVMFile)
            SVMFile.close()
        else:
            print('Failed to recognize the classifier type to load')
            sys.exit()
        
        return
    

