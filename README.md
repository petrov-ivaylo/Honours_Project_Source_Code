# Honours_Project_Source_Code
# This repository contains the source code I created as part of my Honours Project

There are 10 .py documents and 3 .xlsx files. The “Testing_D_star_and_Dijkstra.py" uses the “Platooning_with_D_star_search.py" and “Platooning_with_Dijkstra_search.py" is for the comparison between the D* search and Dijkstra’s search algorithms. The results from these tests are being stored in the “Tests_for_D_star_vs_Dijkstra.xlsx". Similarly, the “Testing_Elzbieta_and_Ivaylo_algorithms_without_charge_restrictions.py" uses the “Ivaylo_D_star_without_charge_restrictions.py" and “Elzbieta_Futkowska_Potential_Field_with_D_star_without_charge_restrictions.py" to compare the Longest Shortest Paths heuristic to the Potential Field Method inspired heuristic, when the energy consumption concept is not considered. The results from these tests are being stored in the “Tests_for_Ivaylo_vs_Elzbieta_without_recharging.xlsx". Finally, the “Ivaylo_vs_Elzbieta_with_recharging_stations.py" uses the files “Ivaylo_D_star_with_recharging.py" and “Elzbieta_Futkowska_Potential_Field_with_D_star_with_recharging.py" to compare the Longest Shortest Paths and the Potential Field Method inspired heuristics, when the energy consumption concept is taken into account. The results from these tests are being stored in the “Tests_for_Ivaylo_vs_Elzbieta_with_recharging.xlsx". 

The software was developed using the software requirements listed below. Hence, it is recommended they to be met in order to ensure that the code will run. If one of the versions of any of the Python packages, for example, is different, then it might not work with the rest.

• Python 3.7.9
• Microsoft Office, more precisely, Excel 2016
Python packages:
– networkx 2.8
– numpy 1.22.2
– pandas 1.4.0
– scipy 1.8.0
– XlsxWriter 3.0.3

Hardware Pre-requisites

The total size of the software folder is 292 KB. In addition, the required memory is not more than
100 MB when the software is running. However, these details vary from the size of the inputs and
the sizes of the calculations, respectively. Therefore, the more RAM is available and the better the
CPU is, the more tests can be done

Every .py file is well-commented and gives much information for all the important classes and variables used
