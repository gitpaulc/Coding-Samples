from django.db import models


class Algorithm(models.Model):
    whichAlgorithm = models.IntegerField(default=0)
    pub_date = models.DateTimeField('date published')
    def __str__(self):
        if self.whichAlgorithm == 0:
            return 'Largest Common Subsequence'
        #else:
        return 'Largest Common Substring'


class InputModel(models.Model):
    algorithm = models.ForeignKey(Algorithm, on_delete=models.CASCADE)
    caption = models.CharField(max_length=200)
    input1 = models.TextField(max_length=200)
    input2 = models.TextField(max_length=200)
    def __str__(self):
        return self.caption
