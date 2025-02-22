import pygame
import sys

# PyGame Colors
WHITE = (255, 255, 255)
BLACK = (42, 51, 53)
RED = (238, 78, 78)
GREEN = (161, 221, 112)
BLUE = (10, 94, 176)
GRAY = (228, 224, 225)

# Initialize PyGame
pygame.init()

# UI class
class UI:
    def __init__(self, width=500, height=750):
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("PyGame UI for PyBullet")
        self.font = pygame.font.Font(None, 30)
        self.font_title = pygame.font.Font(None, 36)
        
        # Default values for sliders
        self.m_value = 1  # Mass m (initial value)
        self.k_value = 100  # Spring constant k (initial value)
        self.r_dot_value = -5 # Angular velocity of R (initial value)
        self.theta_dot_value = 0 # Angular velocity of theta (initial value)
        self.phi_dot_value = 0 # Angular velocity of phi (initial value)
        self.count_value = 1  # Count (initial value)
        
        # Slider ranges and step values
        self.m_min, self.m_max, self.m_step = 1, 5, 0.5
        self.k_min, self.k_max, self.k_step = 10, 1000, 50
        self.r_dot_min, self.r_dot_max, self.r_dot_step = -10, -1, 1
        self.theta_dot_min, self.theta_dot_max, self.theta_dot_step = 0, 1.5, 0.1
        self.phi_dot_min, self.phi_dot_max, self.phi_dot_step = 0, 1.5, 0.1
        self.count_min, self.count_max, self.count_step = 1, 10, 1

    def draw_rounded_rect(self, x, y, width, height, color, radius):
        """Draw rectangle with rounded corners."""
        pygame.draw.rect(self.screen, color, (x, y, width, height), border_radius=radius)

    def draw_slider(self, x, y, width, value, min_value, max_value, step):
        """Draw slider and handle."""
        # Draw slider track
        self.draw_rounded_rect(x, y, width, 20, GRAY, 10)
        
        # Draw slider handle
        slider_pos = x + (value - min_value) * (width - 20) / (max_value - min_value)
        pygame.draw.rect(self.screen, BLUE, (slider_pos, y, 20, 20))

        # Text for the value
        text = self.font.render(f"{value:.1f}", True, BLACK)
        self.screen.blit(text, (x+10, y - 40))

    def draw_button(self, x, y, width, height, text, color, text_color=BLACK):
        """Draw button with text."""
        self.draw_rounded_rect(x, y, width, height, color, 10)
        text_surface = self.font.render(text, True, text_color)
        self.screen.blit(text_surface, (x + (width - text_surface.get_width()) // 2, y + (height - text_surface.get_height()) // 2))

    def handle_events(self):
        """Handle all user events (key presses, mouse clicks, etc.)."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

    def draw_title(self, title, x, y):
        """Draw the title text at the top."""
        title_surface = self.font_title.render(title, True, BLACK)
        self.screen.blit(title_surface, (x, y))

    def text(self, text, x, y):
        text_surface = self.font.render(text, True, BLACK)
        self.screen.blit(text_surface, (x, y))

    def update(self):
        """Update the UI and process events."""
        self.screen.fill(WHITE)
        self.handle_events()

        self.draw_title("Simulation Parameters", 120, 20)  # Title of the UI

        self.text("m:               Kg", 30, 60)
        self.text("k:                     N/m", 30, 160)
        self.text("dr:                  m/s", 30, 260)
        self.text("dtheta:         rad/s", 30, 360)
        self.text("dphi:            rad/s", 30, 460)
        self.text("Count:              ", 30, 560)
        # Draw Sliders
        self.draw_slider(100, 100, 300, self.m_value, self.m_min, self.m_max, self.m_step)
        self.draw_slider(100, 200, 300, self.k_value, self.k_min, self.k_max, self.k_step)
        self.draw_slider(100, 300, 300, self.r_dot_value, self.r_dot_min, self.r_dot_max, self.r_dot_step)
        self.draw_slider(100, 400, 300, self.theta_dot_value, self.theta_dot_min, self.theta_dot_max, self.theta_dot_step)
        self.draw_slider(100, 500, 300, self.phi_dot_value, self.phi_dot_min, self.phi_dot_max, self.phi_dot_step)
        self.draw_slider(100, 600, 300, self.count_value, self.count_min, self.count_max, self.count_step)

        # Draw buttons for m_value adjustments
        self.draw_button(420, 90, 50, 40, "+", GREEN)
        self.draw_button(30, 90, 50, 40, "-", RED)

        # Draw buttons for k_value adjustments
        self.draw_button(420, 190, 50, 40, "+", GREEN)
        self.draw_button(30, 190, 50, 40, "-", RED)

        # Draw buttons for rdot_value adjustments
        self.draw_button(420, 290, 50, 40, "+", GREEN)
        self.draw_button(30, 290, 50, 40, "-", RED)

        self.draw_button(420, 390, 50, 40, "+", GREEN)
        self.draw_button(30, 390, 50, 40, "-", RED)

        self.draw_button(420, 490, 50, 40, "+", GREEN)
        self.draw_button(30, 490, 50, 40, "-", RED)

        # Draw buttons for count_value adjustments
        self.draw_button(420, 590, 50, 40, "+", GREEN)
        self.draw_button(30, 590, 50, 40, "-", RED)

        # Draw Buttons
        self.draw_button(150, 650, 200, 40, "Start Simulation", GREEN)

        pygame.display.update()

    def get_slider_values(self):
        """Return current slider values."""
        return self.m_value, self.k_value, self.r_dot_value, self.theta_dot_value, self.phi_dot_value, self.count_value
    
    def check_button_click(self, x, y, width, height):
        """Check if a button is clicked."""
        mouse_x, mouse_y = pygame.mouse.get_pos()
        mouse_click = pygame.mouse.get_pressed()
        if x <= mouse_x <= x + width and y <= mouse_y <= y + height:
            if mouse_click[0]:  # Left click
                return True
        return False
    