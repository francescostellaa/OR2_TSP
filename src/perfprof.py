import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

# Load the data
data = pd.read_csv('../data/perfprof/perfprof_vns.csv', sep=';')

# Set the first column as the index
data = data.set_index(data.columns[0])

# Function to create performance profile
def create_performance_profile(data):
    # Get the best value (minimum) for each instance (row)
    best_values = data.min(axis=1)
    
    # Calculate the ratio for each algorithm relative to the best
    ratios = pd.DataFrame()
    for col in data.columns:
        ratios[col] = data[col] / best_values
    
    # Create the performance profile with LaTeX-style fonts
    plt.rcParams.update({
        'font.family': 'serif',
        'font.size': 14,
        'text.usetex': False,
        'mathtext.fontset': 'cm'
    })
    
    fig, ax = plt.subplots(figsize=(10, 7))
    
    # Define markers and colors
    markers = ['s', 'o', 'p', 'D', 'v', '+', 'x']
    colors = ['blue', 'red', 'green', 'purple', 'orange', 'brown', 'cyan']
    
    # Sort ratios and calculate cumulative distribution
    for i, col in enumerate(data.columns):
        sorted_ratios = np.sort(ratios[col].values)
        y = np.arange(1, len(sorted_ratios) + 1) / len(sorted_ratios)

        # Prepend (1,0) for proper start
        sorted_ratios = np.insert(sorted_ratios, 0, 1.0)
        y = np.insert(y, 0, 0.0)

        ax.plot(sorted_ratios, y, 
                marker=markers[i % len(markers)], 
                color=colors[i % len(colors)], 
                label=col,
                markersize=7,
                linewidth=1.5)
    
    # Set x-axis limits
    max_ratio = ratios.max().max()
    ax.set_xlim(1.0, min(max_ratio * 1.0, 1.25))
    
    # Set y-axis limits
    ax.set_ylim(0, 1.05)
    
    # Add grid with dashed lines
    ax.grid(True, linestyle='--', alpha=0.7, axis='x')
    
    # Set appropriate x-axis ticks
    tick_spacing = 0.01 if (max_ratio - 1.0) < 0.1 else 0.02
    ax.xaxis.set_major_locator(MultipleLocator(tick_spacing))
    
    # Add labels and legend
    ax.set_xlabel('Cost Ratio', fontsize=16)
    ax.set_ylabel('P(ratio ≤ τ)', fontsize=16)
    ax.legend(loc='lower right', fontsize=14, frameon=True)
    
    # Add a title
    plt.title('Performance Profile of Algorithms', fontsize=16)
    
    return fig, ax

# Create the performance profile
fig, ax = create_performance_profile(data)

# Show the plot
plt.tight_layout()
plt.savefig('performance_profile.png', dpi=300, bbox_inches='tight')
plt.show()

# Display statistics about the algorithms
def display_statistics(data):
    # Get the best value for each instance
    best_values = data.min(axis=1)
    
    # Calculate the ratios
    ratios = pd.DataFrame()
    for col in data.columns:
        ratios[col] = data[col] / best_values
    
    # Calculate statistics
    stats = pd.DataFrame({
        'Mean Ratio': ratios.mean(),
        'Median Ratio': ratios.median(),
        'Min Ratio': ratios.min(),
        'Max Ratio': ratios.max(),
        'Proportion Best': (ratios == 1.0).mean()
    })
    
    return stats.round(4)

stats = display_statistics(data)
print("Performance Statistics:")
print(stats)