namespace WinFormsApp2
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.listView1 = new System.Windows.Forms.ListView();
            this.columnHeader1 = new System.Windows.Forms.ColumnHeader();
            this.columnHeader2 = new System.Windows.Forms.ColumnHeader();
            this.columnHeader3 = new System.Windows.Forms.ColumnHeader();
            this.columnHeader4 = new System.Windows.Forms.ColumnHeader();
            this.columnHeader5 = new System.Windows.Forms.ColumnHeader();
            this.columnHeader6 = new System.Windows.Forms.ColumnHeader();
            this.columnHeader7 = new System.Windows.Forms.ColumnHeader();
            button_z_p = new System.Windows.Forms.Button();
            button_z_n = new System.Windows.Forms.Button();
            button_y_n = new System.Windows.Forms.Button();
            button_y_p = new System.Windows.Forms.Button();
            button_x_p = new System.Windows.Forms.Button();
            button_x_n = new System.Windows.Forms.Button();
            button_rx_n = new System.Windows.Forms.Button();
            button_rx_p = new System.Windows.Forms.Button();
            button_ry_p = new System.Windows.Forms.Button();
            button_ry_n = new System.Windows.Forms.Button();
            button_rz_n = new System.Windows.Forms.Button();
            button_rz_p = new System.Windows.Forms.Button();
            this.cb_lead_through = new System.Windows.Forms.CheckBox();
            label1 = new System.Windows.Forms.Label();
            label2 = new System.Windows.Forms.Label();
            this.SuspendLayout();
            // 
            // listView1
            // 
            this.listView1.Columns.AddRange(new System.Windows.Forms.ColumnHeader[] {
            this.columnHeader1,
            this.columnHeader2,
            this.columnHeader3,
            this.columnHeader4,
            this.columnHeader5,
            this.columnHeader6,
            this.columnHeader7});
            this.listView1.FullRowSelect = true;
            this.listView1.GridLines = true;
            this.listView1.Location = new System.Drawing.Point(12, 12);
            this.listView1.Name = "listView1";
            this.listView1.Size = new System.Drawing.Size(779, 91);
            this.listView1.TabIndex = 2;
            this.listView1.UseCompatibleStateImageBehavior = false;
            this.listView1.View = System.Windows.Forms.View.Details;
            this.listView1.SelectedIndexChanged += new System.EventHandler(this.listView1_SelectedIndexChanged);
            // 
            // columnHeader1
            // 
            this.columnHeader1.Tag = "";
            this.columnHeader1.Text = "IP Address";
            this.columnHeader1.Width = 100;
            // 
            // columnHeader2
            // 
            this.columnHeader2.Text = "ID";
            this.columnHeader2.Width = 100;
            // 
            // columnHeader3
            // 
            this.columnHeader3.Text = "Availability";
            this.columnHeader3.Width = 100;
            // 
            // columnHeader4
            // 
            this.columnHeader4.Text = "Virtual";
            this.columnHeader4.Width = 80;
            // 
            // columnHeader5
            // 
            this.columnHeader5.Text = "System name";
            this.columnHeader5.Width = 120;
            // 
            // columnHeader6
            // 
            this.columnHeader6.Text = "RobotWare Version";
            this.columnHeader6.Width = 100;
            // 
            // columnHeader7
            // 
            this.columnHeader7.Text = "Controller name";
            this.columnHeader7.Width = 100;
            // 
            // button_z_p
            // 
            button_z_p.Location = new System.Drawing.Point(12, 171);
            button_z_p.Name = "button_z_p";
            button_z_p.Size = new System.Drawing.Size(80, 80);
            button_z_p.TabIndex = 4;
            button_z_p.Text = "Z+";
            button_z_p.UseVisualStyleBackColor = true;
            button_z_p.MouseDown += new System.Windows.Forms.MouseEventHandler(button_z_p_MouseDown);
            button_z_p.MouseUp += new System.Windows.Forms.MouseEventHandler(button_z_p_MouseUp);
            // 
            // button_z_n
            // 
            button_z_n.Location = new System.Drawing.Point(12, 343);
            button_z_n.Name = "button_z_n";
            button_z_n.Size = new System.Drawing.Size(80, 80);
            button_z_n.TabIndex = 5;
            button_z_n.Text = "Z-";
            button_z_n.UseVisualStyleBackColor = true;
            button_z_n.MouseDown += new System.Windows.Forms.MouseEventHandler(button_z_n_MouseDown);
            button_z_n.MouseUp += new System.Windows.Forms.MouseEventHandler(button_z_n_MouseUp);
            // 
            // button_y_n
            // 
            button_y_n.Location = new System.Drawing.Point(184, 343);
            button_y_n.Name = "button_y_n";
            button_y_n.Size = new System.Drawing.Size(80, 80);
            button_y_n.TabIndex = 6;
            button_y_n.Text = "Y-";
            button_y_n.UseVisualStyleBackColor = true;
            button_y_n.MouseDown += new System.Windows.Forms.MouseEventHandler(button_y_n_MouseDown);
            button_y_n.MouseUp += new System.Windows.Forms.MouseEventHandler(button_y_n_MouseUp);
            // 
            // button_y_p
            // 
            button_y_p.Location = new System.Drawing.Point(184, 171);
            button_y_p.Name = "button_y_p";
            button_y_p.Size = new System.Drawing.Size(80, 80);
            button_y_p.TabIndex = 7;
            button_y_p.Text = "Y+";
            button_y_p.UseVisualStyleBackColor = true;
            button_y_p.MouseDown += new System.Windows.Forms.MouseEventHandler(button_y_p_MouseDown);
            button_y_p.MouseUp += new System.Windows.Forms.MouseEventHandler(button_y_p_MouseUp);
            // 
            // button_x_p
            // 
            button_x_p.Location = new System.Drawing.Point(270, 257);
            button_x_p.Name = "button_x_p";
            button_x_p.Size = new System.Drawing.Size(80, 80);
            button_x_p.TabIndex = 8;
            button_x_p.Text = "X+";
            button_x_p.UseVisualStyleBackColor = true;
            button_x_p.MouseDown += new System.Windows.Forms.MouseEventHandler(button_x_p_MouseDown);
            button_x_p.MouseUp += new System.Windows.Forms.MouseEventHandler(button_x_p_MouseUp);
            // 
            // button_x_n
            // 
            button_x_n.Location = new System.Drawing.Point(98, 257);
            button_x_n.Name = "button_x_n";
            button_x_n.Size = new System.Drawing.Size(80, 80);
            button_x_n.TabIndex = 9;
            button_x_n.Text = "X-";
            button_x_n.UseVisualStyleBackColor = true;
            button_x_n.MouseDown += new System.Windows.Forms.MouseEventHandler(button_x_n_MouseDown);
            button_x_n.MouseUp += new System.Windows.Forms.MouseEventHandler(button_x_n_MouseUp);
            // 
            // button_rx_n
            // 
            button_rx_n.Location = new System.Drawing.Point(519, 257);
            button_rx_n.Name = "button_rx_n";
            button_rx_n.Size = new System.Drawing.Size(80, 80);
            button_rx_n.TabIndex = 15;
            button_rx_n.Text = "RX-";
            button_rx_n.UseVisualStyleBackColor = true;
            button_rx_n.MouseDown += new System.Windows.Forms.MouseEventHandler(button_rx_n_MouseDown);
            button_rx_n.MouseUp += new System.Windows.Forms.MouseEventHandler(button_rx_n_MouseUp);
            // 
            // button_rx_p
            // 
            button_rx_p.Location = new System.Drawing.Point(691, 257);
            button_rx_p.Name = "button_rx_p";
            button_rx_p.Size = new System.Drawing.Size(80, 80);
            button_rx_p.TabIndex = 14;
            button_rx_p.Text = "RX+";
            button_rx_p.UseVisualStyleBackColor = true;
            button_rx_p.MouseDown += new System.Windows.Forms.MouseEventHandler(button_rx_p_MouseDown);
            button_rx_p.MouseMove += new System.Windows.Forms.MouseEventHandler(button_rx_p_MouseUp);
            // 
            // button_ry_p
            // 
            button_ry_p.Location = new System.Drawing.Point(605, 171);
            button_ry_p.Name = "button_ry_p";
            button_ry_p.Size = new System.Drawing.Size(80, 80);
            button_ry_p.TabIndex = 13;
            button_ry_p.Text = "RY+";
            button_ry_p.UseVisualStyleBackColor = true;
            button_ry_p.MouseDown += new System.Windows.Forms.MouseEventHandler(button_ry_p_MouseDown);
            button_ry_p.MouseUp += new System.Windows.Forms.MouseEventHandler(button_ry_p_MouseUp);
            // 
            // button_ry_n
            // 
            button_ry_n.Location = new System.Drawing.Point(605, 343);
            button_ry_n.Name = "button_ry_n";
            button_ry_n.Size = new System.Drawing.Size(80, 80);
            button_ry_n.TabIndex = 12;
            button_ry_n.Text = "RY-";
            button_ry_n.UseVisualStyleBackColor = true;
            button_ry_n.MouseDown += new System.Windows.Forms.MouseEventHandler(button_ry_n_MouseDown);
            button_ry_n.MouseUp += new System.Windows.Forms.MouseEventHandler(button_ry_n_MouseUp);
            // 
            // button_rz_n
            // 
            button_rz_n.Location = new System.Drawing.Point(433, 343);
            button_rz_n.Name = "button_rz_n";
            button_rz_n.Size = new System.Drawing.Size(80, 80);
            button_rz_n.TabIndex = 11;
            button_rz_n.Text = "RZ-";
            button_rz_n.UseVisualStyleBackColor = true;
            button_rz_n.MouseDown += new System.Windows.Forms.MouseEventHandler(button_rz_n_MouseDown);
            button_rz_n.MouseUp += new System.Windows.Forms.MouseEventHandler(button_rz_n_MouseUp);
            // 
            // button_rz_p
            // 
            button_rz_p.Location = new System.Drawing.Point(433, 171);
            button_rz_p.Name = "button_rz_p";
            button_rz_p.Size = new System.Drawing.Size(80, 80);
            button_rz_p.TabIndex = 10;
            button_rz_p.Text = "RZ+";
            button_rz_p.UseVisualStyleBackColor = true;
            button_rz_p.MouseDown += new System.Windows.Forms.MouseEventHandler(button_rz_p_MouseDown);
            button_rz_p.MouseUp += new System.Windows.Forms.MouseEventHandler(button_rz_p_MouseUp);
            // 
            // cb_lead_through
            // 
            this.cb_lead_through.AutoSize = true;
            this.cb_lead_through.Location = new System.Drawing.Point(71, 134);
            this.cb_lead_through.Name = "cb_lead_through";
            this.cb_lead_through.Size = new System.Drawing.Size(94, 19);
            this.cb_lead_through.TabIndex = 16;
            this.cb_lead_through.Text = "Leadthrough";
            this.cb_lead_through.UseVisualStyleBackColor = true;
            this.cb_lead_through.CheckedChanged += new System.EventHandler(this.cb_lead_through_CheckedChanged);
            // 
            // label1
            // 
            label1.AutoSize = true;
            label1.Location = new System.Drawing.Point(373, 136);
            label1.Name = "label1";
            label1.Size = new System.Drawing.Size(38, 15);
            label1.TabIndex = 17;
            label1.Text = "label1";
            label1.Click += new System.EventHandler(this.label1_Click);
            // 
            // label2
            // 
            label2.AutoSize = true;
            label2.Location = new System.Drawing.Point(131, 478);
            label2.Name = "label2";
            label2.Size = new System.Drawing.Size(38, 15);
            label2.TabIndex = 18;
            label2.Text = "label2";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(803, 574);
            this.Controls.Add(label2);
            this.Controls.Add(label1);
            this.Controls.Add(this.cb_lead_through);
            this.Controls.Add(button_rx_n);
            this.Controls.Add(button_rx_p);
            this.Controls.Add(button_ry_p);
            this.Controls.Add(button_ry_n);
            this.Controls.Add(button_rz_n);
            this.Controls.Add(button_rz_p);
            this.Controls.Add(button_x_n);
            this.Controls.Add(button_x_p);
            this.Controls.Add(button_y_p);
            this.Controls.Add(button_y_n);
            this.Controls.Add(button_z_n);
            this.Controls.Add(button_z_p);
            this.Controls.Add(this.listView1);
            this.Name = "Form1";
            this.Text = "Network scanner";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion
        private ListView listView1;
        private ColumnHeader columnHeader1;
        private ColumnHeader columnHeader2;
        private ColumnHeader columnHeader3;
        private ColumnHeader columnHeader4;
        private ColumnHeader columnHeader5;
        private ColumnHeader columnHeader6;
        private ColumnHeader columnHeader7;
        private static Button button_z_p;
        private static Button button_z_n;
        private static Button button_y_n;
        private static Button button_y_p;
        private static Button button_x_p;
        private static Button button_x_n;
        private static Button button_rx_n;
        private static Button button_rx_p;
        private static Button button_ry_p;
        private static Button button_ry_n;
        private static Button button_rz_n;
        private static Button button_rz_p;
        private CheckBox cb_lead_through;
        private static Label label1;
        private static Label label2;
    }
}