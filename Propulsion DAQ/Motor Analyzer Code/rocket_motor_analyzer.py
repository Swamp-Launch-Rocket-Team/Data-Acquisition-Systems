import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import io
from datetime import datetime
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog
import os
import sys

def analyze_rocket_motor(csv_file_path, output_folder=None, custom_filename=None):
    # Load the data
    try:
        df = pd.read_csv(csv_file_path)
    except Exception as e:
        return f"Error loading CSV file: {str(e)}"
    
    # Convert time from microseconds to seconds
    if 'time' in df.columns:
        df['time_seconds'] = df['time'] / 1000000
    else:
        return "Error: Expected column 'time' not found. Please check your CSV format."
    
    # Check for required columns
    required_columns = ['thrust', 'pressure', 'temperature']
    for col in required_columns:
        if col not in df.columns:
            return f"Error: Expected column '{col}' not found. Please check your CSV format."
    
    # Convert thrust from grams to Newtons (1 gram = 0.00981 N)
    df['thrust_newtons'] = df['thrust'] * 0.00981
    
    # Calculate time differences between data points for integration
    df['delta_t'] = df['time_seconds'].diff().fillna(0)
    
    # Calculate impulse at each time step
    df['impulse_step'] = df['thrust_newtons'] * df['delta_t']
    
    # Calculate total impulse
    total_impulse = df['impulse_step'].sum()
    
    # Calculate average thrust
    avg_thrust = df['thrust_newtons'].mean()
    max_thrust = df['thrust_newtons'].max()
    
    # Calculate burn time (time from first non-zero thrust to last non-zero thrust)
    thrust_threshold = max_thrust * 0.05  # 5% of max thrust as threshold
    thrust_data = df[df['thrust_newtons'] > thrust_threshold]
    if not thrust_data.empty:
        burn_time = thrust_data['time_seconds'].max() - thrust_data['time_seconds'].min()
    else:
        burn_time = 0
    
    # Motor classification based on total impulse
    motor_classes = {
        0: "N/A", 1.26: "A", 2.51: "B", 5.01: "C", 10.01: "D", 20.01: "E",
        40.01: "F", 80.01: "G", 160.01: "H", 320.01: "I", 640.01: "J",
        1280.01: "K", 2560.01: "L", 5120.01: "M", 10240.01: "N", 20480.01: "O"
    }
    
    motor_class = "N/A"
    for impulse_threshold, class_letter in sorted(motor_classes.items()):
        if total_impulse >= impulse_threshold:
            motor_class = class_letter
        else:
            break
    
    # Calculate specific impulse if mass data is available
    specific_impulse = "N/A"
    if 'propellant_mass' in df.columns:
        if df['propellant_mass'].max() > 0:
            specific_impulse = total_impulse / (9.81 * df['propellant_mass'].max())
    
    # Find maximum values
    max_pressure = df['pressure'].max()
    max_temp = df['temperature'].max()

    # Determine output file path
    if custom_filename:
        # Ensure filename ends with .pdf
        if not custom_filename.lower().endswith('.pdf'):
            custom_filename += '.pdf'
        
        if output_folder is None:
            output_file = os.path.join(os.path.dirname(csv_file_path), custom_filename)
        else:
            output_file = os.path.join(output_folder, custom_filename)
    else:
        # Default filename
        if output_folder is None:
            output_file = os.path.join(os.path.dirname(csv_file_path), 'rocket_motor_analysis.pdf')
        else:
            output_file = os.path.join(output_folder, 'rocket_motor_analysis.pdf')
    
    # Create PDF report
    try:
        with PdfPages(output_file) as pdf:
            # Create a title page
            plt.figure(figsize=(8.5, 11))
            plt.axis('off')
            plt.text(0.5, 0.9, "Solid Rocket Motor Analysis Report", ha='center', fontsize=24)
            plt.text(0.5, 0.8, f"Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}", ha='center', fontsize=14)
            plt.text(0.5, 0.75, f"File: {os.path.basename(csv_file_path)}", ha='center', fontsize=12)
            plt.text(0.5, 0.7, f"Motor Classification: {motor_class}", ha='center', fontsize=18)
            plt.text(0.5, 0.6, f"Total Impulse: {total_impulse:.2f} N·s", ha='center', fontsize=14)
            plt.text(0.5, 0.5, f"Burn Time: {burn_time:.2f} seconds", ha='center', fontsize=14)
            plt.text(0.5, 0.4, f"Average Thrust: {avg_thrust:.2f} N", ha='center', fontsize=14)
            plt.text(0.5, 0.3, f"Maximum Thrust: {max_thrust:.2f} N", ha='center', fontsize=14)
            pdf.savefig()
            plt.close()
            
            # Thrust vs. Time plot
            plt.figure(figsize=(8.5, 6))
            plt.plot(df['time_seconds'], df['thrust_newtons'])
            plt.title('Thrust vs. Time')
            plt.xlabel('Time (seconds)')
            plt.ylabel('Thrust (N)')
            plt.grid(True)
            plt.text(0.5, -0.2, f"Maximum Thrust: {max_thrust:.2f} N", 
                     ha='center', transform=plt.gca().transAxes)
            plt.tight_layout()
            pdf.savefig()
            plt.close()
            
            # Pressure vs. Time plot
            plt.figure(figsize=(8.5, 6))
            plt.plot(df['time_seconds'], df['pressure'])
            plt.title('Pressure vs. Time')
            plt.xlabel('Time (seconds)')
            plt.ylabel('Pressure (PSI)')
            plt.grid(True)
            plt.text(0.5, -0.2, f"Maximum Pressure: {max_pressure:.2f} PSI", 
                     ha='center', transform=plt.gca().transAxes)
            plt.tight_layout()
            pdf.savefig()
            plt.close()
            
            # Temperature vs. Time plot
            plt.figure(figsize=(8.5, 6))
            plt.plot(df['time_seconds'], df['temperature'])
            plt.title('Temperature vs. Time')
            plt.xlabel('Time (seconds)')
            plt.ylabel('Temperature (°C)')
            plt.grid(True)
            plt.text(0.5, -0.2, f"Maximum Temperature: {max_temp:.2f} °C", 
                     ha='center', transform=plt.gca().transAxes)
            plt.tight_layout()
            pdf.savefig()
            plt.close()
            
            # Detailed Analysis Page
            plt.figure(figsize=(8.5, 11))
            plt.axis('off')
            plt.text(0.5, 0.95, "Detailed Analysis", ha='center', fontsize=18)
            
            # Create detailed data table
            details = [
                f"Total Impulse: {total_impulse:.2f} N·s",
                f"Motor Classification: {motor_class}",
                f"Burn Time: {burn_time:.2f} seconds",
                f"Average Thrust: {avg_thrust:.2f} N",
                f"Maximum Thrust: {max_thrust:.2f} N",
                f"Maximum Pressure: {max_pressure:.2f} PSI",
                f"Maximum Temperature: {max_temp:.2f} °C",
                f"Specific Impulse: {specific_impulse if isinstance(specific_impulse, str) else f'{specific_impulse:.2f} s'}"
            ]
            
            for i, detail in enumerate(details):
                plt.text(0.1, 0.85 - i*0.05, detail, fontsize=12)
            
            # Add pressure-time integral
            pressure_time_integral = np.trapz(df['pressure'], df['time_seconds'])
            plt.text(0.1, 0.85 - len(details)*0.05, f"Pressure-Time Integral: {pressure_time_integral:.2f} PSI·s", fontsize=12)
            
            # Calculate and display thrust-to-weight ratio if weight data is available
            if 'propellant_mass' in df.columns:
                max_weight = df['propellant_mass'].max() * 0.00981  # Convert to Newtons
                if max_weight > 0:
                    thrust_to_weight = max_thrust / max_weight
                    plt.text(0.1, 0.85 - (len(details)+1)*0.05, f"Thrust-to-Weight Ratio: {thrust_to_weight:.2f}", fontsize=12)
            
            pdf.savefig()
            plt.close()
        
        return f"Analysis complete! Report saved as {output_file}"
    except Exception as e:
        return f"Error creating PDF report: {str(e)}"

class RocketMotorAnalyzerApp:
    def __init__(self, root):
        self.root = root
        root.title("Rocket Motor Analyzer")
        root.geometry("650x450")
        root.configure(bg="#f0f0f0")
        
        # Set window icon if available
        try:
            root.iconbitmap("rocket_icon.ico")  # You'll need to create or find a suitable icon file
        except:
            pass
        
        # Create a frame for content
        main_frame = tk.Frame(root, bg="#f0f0f0")
        main_frame.pack(pady=20, padx=20, fill=tk.BOTH, expand=True)
        
        # Add a title label
        title_label = tk.Label(main_frame, text="Rocket Motor Analysis Tool", 
                              font=("Arial", 16, "bold"), bg="#f0f0f0")
        title_label.pack(pady=10)
        
        # Add instructions
        instructions = tk.Label(main_frame, 
                              text="Upload a CSV file with time, thrust, pressure, and temperature data.",
                              wraplength=500, bg="#f0f0f0", font=("Arial", 10))
        instructions.pack(pady=10)
        
        # Create a frame for the upload button
        upload_frame = tk.Frame(main_frame, bg="#f0f0f0")
        upload_frame.pack(pady=10, fill=tk.X)
        
        upload_label = tk.Label(upload_frame, text="Input CSV:", bg="#f0f0f0")
        upload_label.pack(side=tk.LEFT, padx=5)
        
        self.file_label = tk.Label(upload_frame, text="No file selected", 
                                 width=40, bg="white", relief=tk.SUNKEN)
        self.file_label.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        
        upload_btn = tk.Button(upload_frame, text="Browse...", command=self.browse_file,
                             bg="#4CAF50", fg="white", font=("Arial", 10, "bold"),
                             padx=10)
        upload_btn.pack(side=tk.RIGHT)
        
        # Output folder selection
        output_frame = tk.Frame(main_frame, bg="#f0f0f0")
        output_frame.pack(pady=10, fill=tk.X)
        
        output_label = tk.Label(output_frame, text="Output Folder:", bg="#f0f0f0")
        output_label.pack(side=tk.LEFT, padx=5)
        
        self.output_label = tk.Label(output_frame, text="Default output folder (same as input)", 
                                   width=40, bg="white", relief=tk.SUNKEN)
        self.output_label.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        
        output_btn = tk.Button(output_frame, text="Change...", command=self.browse_output,
                             bg="#2196F3", fg="white", font=("Arial", 10, "bold"),
                             padx=10)
        output_btn.pack(side=tk.RIGHT)
        
        # PDF Name field
        pdf_frame = tk.Frame(main_frame, bg="#f0f0f0")
        pdf_frame.pack(pady=10, fill=tk.X)
        
        pdf_label = tk.Label(pdf_frame, text="PDF Name:", bg="#f0f0f0")
        pdf_label.pack(side=tk.LEFT, padx=5)
        
        self.pdf_name_var = tk.StringVar(value="rocket_motor_analysis")
        self.pdf_name_entry = tk.Entry(pdf_frame, textvariable=self.pdf_name_var, width=40)
        self.pdf_name_entry.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        
        pdf_ext_label = tk.Label(pdf_frame, text=".pdf", bg="#f0f0f0")
        pdf_ext_label.pack(side=tk.LEFT)
        
        # Analyze button
        self.analyze_btn = tk.Button(main_frame, text="Analyze Motor Data", 
                                   command=self.analyze_data,
                                   state=tk.DISABLED, bg="#FF9800", fg="white", 
                                   font=("Arial", 12, "bold"), padx=10, pady=5)
        self.analyze_btn.pack(pady=20)
        
        # Status label
        self.status_label = tk.Label(main_frame, text="", 
                                   wraplength=550, bg="#f0f0f0", fg="#333333")
        self.status_label.pack(pady=10)
        
        # Store selected paths
        self.csv_path = None
        self.output_folder = None
        
    def browse_file(self):
        filetypes = [("CSV files", "*.csv"), ("All files", "*.*")]
        filepath = filedialog.askopenfilename(filetypes=filetypes)
        if filepath:
            self.csv_path = filepath
            self.file_label.config(text=os.path.basename(filepath))
            self.analyze_btn.config(state=tk.NORMAL)
            self.status_label.config(text="")
            
            # Auto-suggest PDF name based on CSV filename
            csv_basename = os.path.splitext(os.path.basename(filepath))[0]
            suggested_name = f"{csv_basename}_analysis"
            self.pdf_name_var.set(suggested_name)
            
    def browse_output(self):
        folder_path = filedialog.askdirectory()
        if folder_path:
            self.output_folder = folder_path
            self.output_label.config(text=os.path.basename(folder_path) or folder_path)
            
    def analyze_data(self):
        if not self.csv_path:
            self.status_label.config(text="Please select a CSV file first.", fg="red")
            return
            
        # Get the custom PDF filename
        pdf_name = self.pdf_name_var.get().strip()
        if not pdf_name:
            self.status_label.config(text="Please enter a name for the PDF file.", fg="red")
            return
            
        self.status_label.config(text="Analyzing data... Please wait.", fg="#333333")
        self.root.update()
        
        result = analyze_rocket_motor(self.csv_path, self.output_folder, pdf_name)
        
        if result.startswith("Error"):
            self.status_label.config(text=result, fg="red")
        else:
            self.status_label.config(text=result, fg="green")
            
            # Determine actual output file path
            if not pdf_name.lower().endswith('.pdf'):
                pdf_name += '.pdf'
                
            if self.output_folder:
                output_file = os.path.join(self.output_folder, pdf_name)
            else:
                output_file = os.path.join(os.path.dirname(self.csv_path), pdf_name)
            
            # Ask if user wants to open the PDF
            if messagebox.askyesno("Analysis Complete", "Analysis complete! Would you like to open the PDF report?"):
                try:
                    os.startfile(output_file)  # Windows only
                except:
                    self.status_label.config(text=f"Report saved but couldn't open PDF automatically.\nLocation: {output_file}", fg="green")

if __name__ == "__main__":
    root = tk.Tk()
    app = RocketMotorAnalyzerApp(root)
    root.mainloop()