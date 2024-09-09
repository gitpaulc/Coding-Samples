
from django.shortcuts import get_object_or_404, render
from django.http import HttpResponse

from .models import Algorithm

def index(request):
    latest_algorithm_list = Algorithm.objects.order_by("-pub_date")[:5]
    context = {"latest_algorithm_list": latest_algorithm_list}
    return render(request, 'computationalBiology/index.html', context)

def algorithm(request, algorithm_id):
    algorithm = get_object_or_404(Algorithm, pk=algorithm_id)
    return render(request, 'computationalBiology/algorithm.html', {'algorithm': algorithm})

def results(request, algorithm_id):
    algorithm = get_object_or_404(Algorithm, pk=algorithm_id)
    response = "You're looking at the results of algorithm %s."
    return HttpResponse(response % algorithm.__str__())

def run(request, algorithm_id):
    algorithm = get_object_or_404(Algorithm, pk=algorithm_id)
    response = "You're running algorithm %s."
    return HttpResponse(response % algorithm.__str__())
