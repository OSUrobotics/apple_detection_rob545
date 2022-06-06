from turtle import color
import pandas as pd

import plotly.express as px
import plotly.graph_objects as go

radi = ["Radius_35cm","Radius_40cm","Radius_45cm","Radius_50cm","Radius_55cm", "Radius_60cm","Radius_65cm"]
colors= ['#6FDE6E', '#E8F086', '#235FA4', '#FF4242']

# pose 4 error
df = pd.read_csv("results_df.csv")
df = df[(df['sphere_pose_num'] == 4)]
df = df[df['apple_pos_err'] < 0.5]
df = df.reset_index()
fig = px.bar(df, x='radi', y='apple_pos_err', text=df['apple_pos_err'].round(3))
fig.update_layout(
    title=dict(
        text="Position Error vs Radi: Sphere Pose 4",
        x=0.5,
        font=dict(
            size=25,
        )
    ),
    font=dict(
        size=20,
    ),
    template = "plotly_white",
    xaxis=dict(
        title="Radii",
        #dtick=1,
        #tickfont = dict(size=18)
    ),
    yaxis=dict(
        title="Absolute Position Error (m)",
        #tickfont = dict(size=18)
    ),
    legend=dict(
        title={'text': None},
        yanchor="top",
        y=1,
        xanchor="right",
        x=0.9,
        bgcolor  = 'rgba(255,255,255,0.1)',
    ),
    width=800,
    height=500
)
fig.write_image("results/pos_error_v_radii-sphere_pos_4.png")
fig.show()


# err across sphere points
df = pd.read_csv("results_df.csv")
df = df[(df['pose_num'] != 'averaged_pos') & (df['pose_num'] != 'combined_icp')]
df = df[df['apple_pos_err'] < 0.5]
df = df.groupby('sphere_pose_num').mean()
df = df.reset_index()
fig = px.bar(df, x='sphere_pose_num', y='apple_pos_err', text=df['apple_pos_err'].round(3))
fig.update_layout(
    title=dict(
        text="Position Error vs Sphere Pose",
        x=0.5,
        font=dict(
            size=25,
        )
    ),
    font=dict(
        size=20,
    ),
    template = "plotly_white",
    xaxis=dict(
        title="Sphere Pose Number",
        dtick=1,
        #tickfont = dict(size=18)
    ),
    yaxis=dict(
        title="Absolute Position Error (m)",
        #tickfont = dict(size=18)
    ),
    legend=dict(
        title={'text': None},
        yanchor="top",
        y=1,
        xanchor="right",
        x=0.9,
        bgcolor  = 'rgba(255,255,255,0.1)',
    ),
    width=800,
    height=500
)
fig.write_image("results/pos_error_v_sphere_pos.png")
fig.show()

# combined vs averaged err
# df = pd.read_csv("results_df.csv")
# df = df[(df['pose_num'] == 'averaged_pos') | (df['pose_num'] == 'combined_icp')]
# df = df[df['apple_pos_err'] < 0.5]
# fig = px.bar(df, x='radi', y='apple_pos_err', color='pose_num', barmode='group')
# fig.update_layout(
#     title=dict(
#         text="Position Error vs Radi: Combined ICP and Averaged Position",
#         x=0.5,
#         font=dict(
#             size=45,
#         )
#     ),
#     font=dict(
#         size=38,
#     ),
#     template = "plotly_white",
#     xaxis=dict(
#         title="Radii",
#         #dtick=1,
#         #tickfont = dict(size=18)
#     ),
#     yaxis=dict(
#         title="Absolute Position Error (m)",
#         #tickfont = dict(size=18)
#     ),
#     legend=dict(
#         title={'text': None},
#         yanchor="top",
#         y=0.97,
#         xanchor="right",
#         x=0.97,
#         bgcolor  = 'rgba(255,255,255,0.1)',
#     ),
# )
# fig.show()

# # conbined icp plot
# color blind colors: 
colors= ['#6FDE6E', '#E8F086', '#235FA4', '#FF4242']
df = pd.read_csv("results_df.csv")
df = df[(df['pose_num'] == 'combined_icp')]
df = df[df['apple_pos_err'] < 0.5]
fig = go.Figure(go.Bar(x=df['radi'], y=df['err_x'], name='Error X'))
fig.add_trace(go.Bar(x=df['radi'], y=df['err_y'], name='Error Y'))
fig.add_trace(go.Bar(x=df['radi'], y=df['err_z'], name='Error Z'))
fig.add_trace(go.Bar(x=df['radi'], y=df['apple_pos_err'], name='Abs Error'))
fig.update_layout(barmode='group', xaxis={'categoryorder':'array', 'categoryarray':radi})
fig.update_layout(
    title=dict(
        text="Error vs Radi: Combined ICP",
        x=0.5,
        font=dict(
            size=25,
        )
    ),
    font=dict(
        size=20,
    ),
    template = "plotly_white",
    xaxis=dict(
        title="Radii",
        #dtick=1,
        #tickfont = dict(size=18)
    ),
    yaxis=dict(
        title="Absolute Position Error (m)",
        #tickfont = dict(size=18)
    ),
    legend=dict(
        title={'text': None},
        yanchor="top",
        y=0.97,
        xanchor="right",
        x=0.97,
        bgcolor  = 'rgba(255,255,255,0.1)',
    ),
    width=800,
    height=500
)
fig.write_image("results/pos_error_v_radi-combined_icp.png")
fig.show()

# # averaged x,y,z plot
df = pd.read_csv("results_df.csv")
df = df[(df['pose_num'] == 'averaged_pos')]
df = df[df['apple_pos_err'] < 0.5]
fig = go.Figure(go.Bar(x=df['radi'], y=df['err_x'], name='Error X'))
fig.add_trace(go.Bar(x=df['radi'], y=df['err_y'], name='Error Y'))
fig.add_trace(go.Bar(x=df['radi'], y=df['err_z'], name='Error Z'))
fig.add_trace(go.Bar(x=df['radi'], y=df['apple_pos_err'], name='Abs Error'))
fig.update_layout(barmode='group', xaxis={'categoryorder':'array', 'categoryarray':radi}, )
fig.update_layout(
    title=dict(
        text="Error vs Radi: Averaged Position",
        x=0.5,
        font=dict(
            size=25,
        )
    ),
    font=dict(
        size=20,
    ),
    template = "plotly_white",
    xaxis=dict(
        title="Radii",
        #dtick=1,
        #tickfont = dict(size=18)
    ),
    yaxis=dict(
        title="Absolute Position Error (m)",
        #tickfont = dict(size=18)
    ),
    legend=dict(
        title={'text': None},
        yanchor="top",
        y=1,
        xanchor="right",
        x=0.9,
        bgcolor  = 'rgba(255,255,255,0.1)',
    ),
    width=800,
    height=500
)
fig.write_image("results/pos_error_v_radi-averaged_pos.png")
fig.show()

