from django.shortcuts import render


def test_updates(request):
    return render(request, 'test_updates.html', {})
