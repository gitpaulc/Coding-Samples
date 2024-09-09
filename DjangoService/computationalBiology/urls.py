
from django.urls import path

from . import views

app_name = 'computationalBiology'
urlpatterns = [
    path('', views.index, name='index'),
    path('<int:algorithm_id>/', views.algorithm, name='algorithm'),
    path('<int:algorithm_id>/results/', views.results, name='results'),
    path('<int:algorithm_id>/run/', views.run, name='run'),
]
