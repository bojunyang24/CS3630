#!/usr/bin/env python

##############
#### Your name: Bojun Yang
##############

import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color, measure
import ransac_score
import matplotlib.pyplot as plt
import joblib

class ImageClassifier:
    
    def __init__(self):
        self.classifier = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)
        
        #create one large array of image data
        data = io.concatenate_images(ic)
        
        #extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]
        
        return(data,labels)

    def extract_image_features(self, data):
        # Please do not modify the header above
        
        # extract feature vector from image data
        feature_data = []
        y_max, x_max, _ = data[0].shape
        for d in data:
            greyscaled = filters.gaussian(d[:,:,0], sigma=1)
            hog_descriptor = feature.hog(greyscaled, orientations=10, pixels_per_cell=(64,64), cells_per_block=(2,1), block_norm='L2-Hys', visualize=False)
            feature_data.append(hog_descriptor)
            ### visualize
            # hog_descriptor, hog_image = feature.hog(greyscaled, orientations=9, pixels_per_cell=(8,8), cells_per_block=(4,4), block_norm='L1', visualize=True)
            # fig, (ax1, ax2) = plt.subplots(nrows = 1, ncols = 2)
            # ax1.imshow(greyscaled, cmap=plt.cm.gray)
            # ax1.set_title('greyscaled image')
            # ax2.imshow(hog_image, cmap=plt.cm.gray)
            # ax2.set_title('edge detection')
            # plt.show()
        
        feature_data = np.vstack(feature_data)

        # Please do not modify the return type below
        return(feature_data)

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above
        
        # train model and save the trained model to self.classifier
        
        linear_classifier = svm.LinearSVC()
        linear_classifier.fit(train_data, train_labels)

        self.classifier = linear_classifier
        joblib.dump(self.classifier, 'classifier.pkl')

    def predict_labels(self, data):
        # Please do not modify the header

        # predict labels of test data using trained model in self.classifier
        # the code below expects output to be stored in predicted_labels
        
        predicted_labels = self.classifier.predict(data)

        # Please do not modify the return type below
        return predicted_labels

    def line_fitting(self, data):
        # Please do not modify the header

        # fit a line the to arena wall using RANSAC
        # return two lists containing slopes and y intercepts of the line
        y_max, x_max, _ = data[0].shape
        slope = []
        intercept = []
        for d in data:
            greyscaled = filters.gaussian(d[:,:,0], sigma=3)
            edges = feature.canny(greyscaled, sigma = 3)
            ransac_input = np.argwhere(edges)

            ransac_model, inliers = measure.ransac(ransac_input, measure.LineModelND, 2, 1)
            origin, direction = ransac_model.params
            m = direction[0] / direction[1]
            y = origin[0] - m * origin[1]

            slope.append(m)
            intercept.append(y)

            ### visualize
            # fig, (ax1, ax2) = plt.subplots(nrows = 1, ncols = 2)
            # ax1.imshow(greyscaled, cmap=plt.cm.gray)
            # ax1.set_title('greyscaled image')
            # ax2.imshow(edges, cmap=plt.cm.gray)
            # ax2.set_title('edge detection')
            # ax1.plot([0, x_max], [y, y + m * x_max], '-r', label="ransac line fit")
            # plt.show()

            ### console debugger
            # print('m: {} \ny: {}'.format(m, y))

        # Please do not modify the return type below
        return slope, intercept
        
def main():

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')
    (wall_raw, _) = img_clf.load_data_from_folder('./wall/')
    
    # convert images into features
    # train_data = img_clf.extract_image_features(train_raw)
    # test_data = img_clf.extract_image_features(test_raw)

    try:
        train_data = np.load('training_data.npy')
    except FileNotFoundError:
        train_data = img_clf.extract_image_features(train_raw)
        np.save('training_data',train_data)

    try:
        test_data = np.load('testing_data.npy')
    except FileNotFoundError:
        test_data = img_clf.extract_image_features(test_raw)
        np.save('testing_data',test_data)
    
    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)
    predicted_labels = img_clf.predict_labels(train_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(train_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))
    
    # test model
    predicted_labels = img_clf.predict_labels(test_data)
    print("\nTest results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(test_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))

    # ransac
    print("\nRANSAC results")
    print("=============================")
    s, i = img_clf.line_fitting(wall_raw)
    print(f"Line Fitting Score: {ransac_score.score(s,i)}/10")

if __name__ == "__main__":
    main()
