# README

Project 4: Bag of Words 

Abstract:
The objective of Project 4 is detect objects of a given class using a generalized hough transform approach, a bin voting method. Given a set of training images which are examples of a car dataset, a bag of words method is used to extract key features. Harris corner detection is used to find key features and an image patch centered at these corners are recorded. From there the displacement vectors are stored between the patch and the training image center. This is repeated for the training images and with the visual vocabulary, the object position is used to index votes where each visual word is expected to be a car part. Each part uses the recorded displacement and using the peaks in the voting space leads to one or more objects being detected.
