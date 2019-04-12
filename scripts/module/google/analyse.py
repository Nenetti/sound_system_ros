# -*- coding: utf-8 -*-
import csv
import nltk


class Analyse:
    categories = {}

    def __init__(self, path):
        with open(path, "r") as file:
            reader = csv.reader(file)
            for line in reader:
                if len(line) > 1 and line[0] != "":
                    self.categories[line[0]] = Category(line[0], line[1:])

    @staticmethod
    def tagged(text):
        tokens = nltk.word_tokenize(text)
        tagged = nltk.pos_tag(tokens)
        return tagged

    def get_nn(self, tagged):
        result = []
        for tag in tagged:
            if "NN" in tag[1]:
                result.append(tag)
        return result

    def get_vb(self, tagged):
        result = []
        for tag in tagged:
            if "VB" in tag[1]:
                result.append(tag)
        return result

    def get_location(self, nns):
        for word in self.categories.get("location").words:
            for tag in nns:
                if tag[0] == word:
                    return word
        return None


class Category:

    def __init__(self, name, words):
        self.name = name
        self.words = words
