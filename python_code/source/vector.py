from dataclasses import dataclass
import math

@dataclass
class Vector:
     
    def __init__(self, head, tail):
        self.head = head # first vertex of line segment
        self.tail = tail # second vertex of line segment
        self.component_form = self._calculate_component_form()
        self.magnitude = self._calculate_magnitude()
        
    def _calculate_magnitude(self):
        ui, vj = self.component_form
        self.magnitude = math.sqrt(ui**2 + vj**2)
        
    def _calculate_component_form(self):
        x1, y1 = self.head
        x2, y2 = self.tail
        
        component_form = []
        component_form.append(x2 - x1)
        component_form.append(y2 - y1)
        return component_form # return the component form of the vector
       

        
        