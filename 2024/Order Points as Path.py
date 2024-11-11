# -*- coding: utf-8 -*-

"""
***************************************************************************
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
***************************************************************************
"""

from qgis.PyQt.QtCore import QCoreApplication
from qgis.core import (QgsProcessing,
                       QgsFeatureSink,
                       QgsProcessingException,
                       QgsProcessingAlgorithm,
                       QgsProcessingParameterFeatureSource,
                       QgsProcessingParameterFeatureSink)
from qgis import processing

##########--MUCH OF THIS CODE IS BOILERPLATE AUTOMATICALLY INCLUDED BY QGIS; MY ALGORITHM WILL BE SECTIONED OFF BY A FLAG SIMILAR TO THIS ONE--##########

class ExampleProcessingAlgorithm(QgsProcessingAlgorithm):
    # Constants used to refer to parameters and outputs. They will be
    # used when calling the algorithm from another algorithm, or when
    # calling from the QGIS console.

    INPUT = 'INPUT'
    OUTPUT = 'OUTPUT'

    def tr(self, string):
        return QCoreApplication.translate('Processing', string)

    def createInstance(self):
        return ExampleProcessingAlgorithm()

    def name(self):
        return 'orderbypath'

    def displayName(self):
        return self.tr('Order By Path')

    def group(self):
        return self.tr('Autonomous Aquatics')

    def groupId(self):
        return 'autonomousaquatics'

    def shortHelpString(self):
        return self.tr("Orders points such that they can be used as a path for the vehicle")

    def initAlgorithm(self, config=None):
        # We add the input vector features source. It can have any kind of
        self.addParameter(
            QgsProcessingParameterFeatureSource(
                self.INPUT,
                self.tr('Input layer'),
                [QgsProcessing.TypeVectorPoint]
            )
        )

        # We add a feature sink in which to store our processed features (this
        # usually takes the form of a newly created vector layer when the
        # algorithm is run in QGIS).
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT,
                self.tr('Output layer')
            )
        )
        
    def processAlgorithm(self, parameters, context, feedback):
        # Retrieve the feature source and sink. The 'dest_id' variable is used
        # to uniquely identify the feature sink, and must be included in the
        # dictionary returned by the processAlgorithm function.
        source = self.parameterAsSource(
            parameters,
            self.INPUT,
            context
        )
        
        # If source was not found, throw an exception to indicate that the algorithm
        # encountered a fatal error. The exception text can be any string, but in this
        # case we use the pre-built invalidSourceError method to return a standard
        # helper text for when a source cannot be evaluated
        if source is None:
            raise QgsProcessingException(self.invalidSourceError(parameters, self.INPUT))

        (sink, dest_id) = self.parameterAsSink(
            parameters,
            self.OUTPUT,
            context,
            source.fields(),
            source.wkbType(),
            source.sourceCrs()
        )

        # Send some information to the user
        feedback.pushInfo('CRS is {}'.format(source.sourceCrs().authid()))

        # If sink was not created, throw an exception to indicate that the algorithm
        # encountered a fatal error. The exception text can be any string, but in this
        # case we use the pre-built invalidSinkError method to return a standard
        # helper text for when a sink cannot be evaluated
        if sink is None:
            raise QgsProcessingException(self.invalidSinkError(parameters, self.OUTPUT))

        # Compute the number of steps to display within the progress bar and
        # get features from source
        total = 100.0 / source.featureCount() if source.featureCount() else 0
        features = list(source.getFeatures())
        
        ##########--MY ALGORITHM BEGINS HERE--##########

        featureList = []
        index = 0
        
        while len(features) > 0:
            feedback.pushInfo("Rep:" + str(index))
            #Put the next feature into our own feature list
            featureList.append([features[0]])
            #If any features in the feature list have the same X coordinate as our current one, we append it under our current entry
            for i, feature in enumerate(features):
                if i == 0: continue
                feedback.pushInfo("Sub-rep:" + str(i))

                if feedback.isCanceled():
                    break
                
                if feature['left'] == featureList[index][0]['left']:
                    featureList[index].append(feature)
                else:
                    continue
            #Then, we remove it from the original features list
            for x in featureList[index]:
                feedback.pushInfo("Del-rep:" + str(x))
                features.remove(x)
            index += 1
        
        #Bubble sort each row, and then bubble sort the rows by left-most to right-most, inverting the order of the row sort for every other
        #This puts them in the zig-zag order visible in the maker portfolio documentation
        swapped = True
        while swapped:
            swapped = False
            for i in range(len(featureList)-1):
                if featureList[i][0]['left'] < featureList[i+1][0]['left']:
                    feedback.pushInfo("Swapping" + str(featureList[i][0]['left']) + ", " + str(featureList[i+1][0]['left']))
                    featureList[i], featureList[i+1] = featureList[i+1], featureList[i]
                    swapped = True
                    feedback.pushInfo("Post-Swap" + str(featureList[i][0]['left']) + ", " + str(featureList[i+1][0]['left']))
        
        for j, column in enumerate(featureList):
            swapped = True
            while swapped:
                swapped = False
                for i in range(len(column)-1):
                    if j%2:
                        if column[i]['top'] < column[i+1]['top']:
                            column[i], column[i+1] = column[i+1], column[i]
                            swapped = True
                    else:
                        if column[i]['top'] > column[i+1]['top']:
                            column[i], column[i+1] = column[i+1], column[i]
                            swapped = True
        
        for column in featureList:
            for feature in column:
                sink.addFeature(feature, QgsFeatureSink.FastInsert)
                # Update the progress bar
                feedback.setProgress(int(i * total))

        return {self.OUTPUT: dest_id}
