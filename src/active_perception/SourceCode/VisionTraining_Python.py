"""
The module to train supervised classifiers
"""

#importing libraries
import sklearn
from sklearn import model_selection
from sklearn import svm
from sklearn.utils import shuffle
from sklearn.externals import joblib
from distutils.util import strtobool

import numpy as np


class Feature_Classifier_Training:
    """
    The class to train classifiers tailored for dealing with engineered features
    """
    def SupportVectorMachine (self, sampleFeatureMatrix, sampleLabelCodes, classifier_source_folder, classifier_folder_name, SVMKernelType, DST_Enabled):
        """
        Method to train a Support Vector Machine
        """
        
        #Hint: Shuffling training data is not needed for Support Vector Machines, since they are optimization-based methods!
        
        #Making the label vector a 1D array by raveling
        sampleLabelCodes_raveled = sampleLabelCodes.ravel()
        
        #setting the classifier configurations
        CacheSize = 5000
        useProbability = True
        SVMMultiClassComparisonMode = 'ovr'
        classWeight = 'balanced' # None
        SVMKernel = SVMKernelType
        
        #finding the optimal classifier parameters (C and Gamma)
        C_range = np.logspace(-2, 10, 13)
        gamma_range = np.logspace(-9, 3, 13)
        param_grid = dict(gamma = gamma_range, C = C_range)
        CrossValidationSettings = model_selection.KFold(n_splits = 4, shuffle = True)
        SVMToBeOptimized = svm.SVC(kernel = SVMKernel, decision_function_shape = SVMMultiClassComparisonMode, probability = useProbability, cache_size = CacheSize, class_weight = classWeight)
        gridOfClassifiers = model_selection.GridSearchCV(SVMToBeOptimized, param_grid = param_grid, cv = CrossValidationSettings, refit = True, n_jobs = -1, verbose = 1)
        gridOfClassifiers.fit(sampleFeatureMatrix, sampleLabelCodes_raveled)
        
        #defining the SVM classifier
        SVMClassifier = gridOfClassifiers.best_estimator_
        
        #saving the trained classifier
        if (DST_Enabled):
            fileAddress = classifier_source_folder + '/' + classifier_folder_name + '/Dempster-Shafer/SVM.pkl'
        else:
            fileAddress = classifier_source_folder + '/' + classifier_folder_name + '/SVM.pkl'
        
        SVMFile = open(fileAddress, "wb+")
        joblib.dump(SVMClassifier, SVMFile)
        
        return
