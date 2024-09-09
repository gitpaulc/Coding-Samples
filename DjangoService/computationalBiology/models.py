
from django.db import models

class Algorithm(models.Model):
    whichAlgorithm = models.IntegerField(default=0)
    pub_date = models.DateTimeField('date published')
    def __str__(self):
        if self.whichAlgorithm == 0:
            return 'Longest Common Subsequence'
        #else:
            return 'Longest Common Substring'


class InputString(models.Model):
    algorithm = models.ForeignKey(Algorithm, on_delete=models.CASCADE)
    caption = models.CharField(max_length=200)
    # inputField should contain default input.
    inputField = models.TextField(max_length=1024)
    def __str__(self):
        return self.caption
