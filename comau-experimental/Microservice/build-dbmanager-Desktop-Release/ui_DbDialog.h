/********************************************************************************
** Form generated from reading UI file 'DbDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DBDIALOG_H
#define UI_DBDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_DbDialog
{
public:
    QGridLayout *gridLayout;
    QStackedWidget *container;
    QWidget *containerPage1;
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout_4;
    QWidget *formpanel;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_7;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *closeButton_2;
    QSpacerItem *verticalSpacer_8;
    QWidget *server_Widget;
    QHBoxLayout *userWidget;
    QLabel *server_label;
    QWidget *servenameContainer;
    QGridLayout *gridLayout_5;
    QLineEdit *server_input;
    QSpacerItem *verticalSpacer_7;
    QWidget *dbname_Widget;
    QHBoxLayout *passWidget;
    QLabel *db_label;
    QWidget *dbContainer;
    QGridLayout *gridLayout_6;
    QLineEdit *dbname_input;
    QSpacerItem *verticalSpacer_4;
    QWidget *user_Widget;
    QHBoxLayout *horizontalLayout;
    QLabel *user_label;
    QWidget *userContainer;
    QGridLayout *gridLayout_7;
    QLineEdit *user_input;
    QSpacerItem *verticalSpacer_5;
    QWidget *pass_Widget;
    QHBoxLayout *horizontalLayout_2;
    QLabel *passwd_label;
    QWidget *passContainer;
    QGridLayout *gridLayout_8;
    QLineEdit *passwd_input;
    QSpacerItem *verticalSpacer_9;
    QWidget *widget_3;
    QHBoxLayout *horizontalLayout_6;
    QLabel *passwd_label_2;
    QGroupBox *trustedGB;
    QHBoxLayout *horizontalLayout_5;
    QRadioButton *yesRB;
    QRadioButton *noRB;
    QSpacerItem *verticalSpacer_6;
    QWidget *widget;
    QHBoxLayout *horizontalLayout_4;
    QSpacerItem *horizontalSpacer;
    QWidget *widget_2;
    QGridLayout *gridLayout_3;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *reset_button;
    QPushButton *connect_button;
    QWidget *containerPage2;
    QGridLayout *gridLayout_15;
    QGridLayout *gridLayout_9;
    QWidget *formpanel_2;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_8;
    QSpacerItem *horizontalSpacer_4;
    QPushButton *closeButton_3;
    QWidget *dbname_Widget_2;
    QHBoxLayout *passWidget_2;
    QWidget *servenameContainer_2;
    QGridLayout *gridLayout_10;
    QLineEdit *dbPathLineEdit;
    QLabel *server_label_2;
    QWidget *widget_5;
    QHBoxLayout *horizontalLayout_12;
    QSpacerItem *horizontalSpacer_5;
    QWidget *widget_6;
    QGridLayout *gridLayout_14;
    QSpacerItem *horizontalSpacer_6;
    QPushButton *reset_button_2;
    QPushButton *connect_button_2;
    QSpacerItem *verticalSpacer_2;
    QSpacerItem *verticalSpacer_3;
    QSpacerItem *verticalSpacer_10;

    void setupUi(QDialog *DbDialog)
    {
        if (DbDialog->objectName().isEmpty())
            DbDialog->setObjectName(QStringLiteral("DbDialog"));
        DbDialog->resize(736, 721);
        DbDialog->setStyleSheet(QLatin1String("#containerPage1{background-color:transparent;}\n"
"#containerPage2{background-color:transparent;}\n"
"QMessageBox{\n"
"background-color: rgb(0, 0, 0);color:white;}\n"
"QMessageBox QPushButton{color:white; background-color:transparent;}\n"
"QMessageBox QLabel{color:white; background-color:transparent;}\n"
"#formpanel {\n"
"  background-color:rgb(210, 210, 210);border-radius:30px;\n"
"border:2px solid rgb(18, 77, 172);\n"
"}\n"
"#formpanel_2 {\n"
"  background-color:rgb(210, 210, 210);border-radius:30px;\n"
"border:2px solid rgb(18, 77, 172);\n"
"}\n"
"QLineEdit{\n"
"font: 16pt \"Leelawadee UI\";\n"
"border:1px solid rgb(18, 77, 172);\n"
"border-radius:10px;\n"
"color:rgb(18, 77, 172);}\n"
"QLabel{color:rgb(18, 77, 172); font:Bold 16pt \"Leelawadee UI\";}\n"
"QPushButton{\n"
"background-color:rgb(18, 77, 172);\n"
"border:1px solid rgb(37, 84, 255);\n"
"color:white;\n"
"border-radius: 16px;\n"
"font-size:17px;\n"
" font: bold 14px \"Leelawadee UI\";\n"
"}\n"
"\n"
"QPushButton:hover{\n"
"background-color:blue;\n"
""
                        "border:1px solid rgb(37, 84, 255);\n"
"color:white;\n"
"border-radius: 16px;\n"
"font-size:17px;\n"
" font: bold 14px \"Leelawadee\";\n"
"}"));
        gridLayout = new QGridLayout(DbDialog);
        gridLayout->setSpacing(0);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 5, 0, 5);
        container = new QStackedWidget(DbDialog);
        container->setObjectName(QStringLiteral("container"));
        container->setStyleSheet(QStringLiteral(""));
        containerPage1 = new QWidget();
        containerPage1->setObjectName(QStringLiteral("containerPage1"));
        gridLayout_2 = new QGridLayout(containerPage1);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        gridLayout_4 = new QGridLayout();
        gridLayout_4->setSpacing(6);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(2, 2, 2, 2);
        formpanel = new QWidget(containerPage1);
        formpanel->setObjectName(QStringLiteral("formpanel"));
        verticalLayout = new QVBoxLayout(formpanel);
        verticalLayout->setSpacing(0);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(20, 10, 20, 20);
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        horizontalLayout_7->setContentsMargins(0, -1, -1, -1);
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_3);

        closeButton_2 = new QPushButton(formpanel);
        closeButton_2->setObjectName(QStringLiteral("closeButton_2"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(30);
        sizePolicy.setVerticalStretch(30);
        sizePolicy.setHeightForWidth(closeButton_2->sizePolicy().hasHeightForWidth());
        closeButton_2->setSizePolicy(sizePolicy);
        closeButton_2->setMinimumSize(QSize(20, 20));
        closeButton_2->setStyleSheet(QLatin1String("#closeButton_2{background-color: transparent;\n"
"font: Bold 14pt \"Leelawadee\";\n"
"color:red;}\n"
"#closeButton_2:hover{background-color: red;\n"
"font: Bold 14pt \"Leelawadee\";\n"
"color:white;}"));

        horizontalLayout_7->addWidget(closeButton_2);


        verticalLayout->addLayout(horizontalLayout_7);

        verticalSpacer_8 = new QSpacerItem(20, 15, QSizePolicy::Minimum, QSizePolicy::Ignored);

        verticalLayout->addItem(verticalSpacer_8);

        server_Widget = new QWidget(formpanel);
        server_Widget->setObjectName(QStringLiteral("server_Widget"));
        userWidget = new QHBoxLayout(server_Widget);
        userWidget->setSpacing(20);
        userWidget->setContentsMargins(11, 11, 11, 11);
        userWidget->setObjectName(QStringLiteral("userWidget"));
        userWidget->setContentsMargins(0, 0, 0, 0);
        server_label = new QLabel(server_Widget);
        server_label->setObjectName(QStringLiteral("server_label"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(server_label->sizePolicy().hasHeightForWidth());
        server_label->setSizePolicy(sizePolicy1);
        QFont font;
        font.setFamily(QStringLiteral("Leelawadee UI"));
        font.setPointSize(16);
        font.setBold(true);
        font.setItalic(false);
        font.setWeight(75);
        server_label->setFont(font);
        server_label->setStyleSheet(QStringLiteral(""));

        userWidget->addWidget(server_label);

        servenameContainer = new QWidget(server_Widget);
        servenameContainer->setObjectName(QStringLiteral("servenameContainer"));
        gridLayout_5 = new QGridLayout(servenameContainer);
        gridLayout_5->setSpacing(0);
        gridLayout_5->setContentsMargins(11, 11, 11, 11);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        gridLayout_5->setContentsMargins(5, 5, 5, 5);
        server_input = new QLineEdit(servenameContainer);
        server_input->setObjectName(QStringLiteral("server_input"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(server_input->sizePolicy().hasHeightForWidth());
        server_input->setSizePolicy(sizePolicy2);
        QFont font1;
        font1.setFamily(QStringLiteral("Leelawadee UI"));
        font1.setPointSize(16);
        font1.setBold(false);
        font1.setItalic(false);
        font1.setWeight(50);
        server_input->setFont(font1);
        server_input->setAlignment(Qt::AlignJustify|Qt::AlignVCenter);

        gridLayout_5->addWidget(server_input, 0, 0, 1, 1);


        userWidget->addWidget(servenameContainer);

        userWidget->setStretch(0, 1);
        userWidget->setStretch(1, 3);

        verticalLayout->addWidget(server_Widget);

        verticalSpacer_7 = new QSpacerItem(20, 15, QSizePolicy::Minimum, QSizePolicy::Fixed);

        verticalLayout->addItem(verticalSpacer_7);

        dbname_Widget = new QWidget(formpanel);
        dbname_Widget->setObjectName(QStringLiteral("dbname_Widget"));
        passWidget = new QHBoxLayout(dbname_Widget);
        passWidget->setSpacing(20);
        passWidget->setContentsMargins(11, 11, 11, 11);
        passWidget->setObjectName(QStringLiteral("passWidget"));
        passWidget->setContentsMargins(0, 0, 0, 0);
        db_label = new QLabel(dbname_Widget);
        db_label->setObjectName(QStringLiteral("db_label"));
        sizePolicy1.setHeightForWidth(db_label->sizePolicy().hasHeightForWidth());
        db_label->setSizePolicy(sizePolicy1);
        db_label->setFont(font);
        db_label->setStyleSheet(QStringLiteral(""));

        passWidget->addWidget(db_label);

        dbContainer = new QWidget(dbname_Widget);
        dbContainer->setObjectName(QStringLiteral("dbContainer"));
        gridLayout_6 = new QGridLayout(dbContainer);
        gridLayout_6->setSpacing(0);
        gridLayout_6->setContentsMargins(11, 11, 11, 11);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        gridLayout_6->setContentsMargins(5, 5, 5, 5);
        dbname_input = new QLineEdit(dbContainer);
        dbname_input->setObjectName(QStringLiteral("dbname_input"));
        sizePolicy2.setHeightForWidth(dbname_input->sizePolicy().hasHeightForWidth());
        dbname_input->setSizePolicy(sizePolicy2);
        dbname_input->setFont(font1);

        gridLayout_6->addWidget(dbname_input, 0, 0, 1, 1);


        passWidget->addWidget(dbContainer);

        passWidget->setStretch(0, 1);
        passWidget->setStretch(1, 3);

        verticalLayout->addWidget(dbname_Widget);

        verticalSpacer_4 = new QSpacerItem(20, 15, QSizePolicy::Minimum, QSizePolicy::Fixed);

        verticalLayout->addItem(verticalSpacer_4);

        user_Widget = new QWidget(formpanel);
        user_Widget->setObjectName(QStringLiteral("user_Widget"));
        horizontalLayout = new QHBoxLayout(user_Widget);
        horizontalLayout->setSpacing(20);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        user_label = new QLabel(user_Widget);
        user_label->setObjectName(QStringLiteral("user_label"));
        sizePolicy1.setHeightForWidth(user_label->sizePolicy().hasHeightForWidth());
        user_label->setSizePolicy(sizePolicy1);
        user_label->setFont(font);
        user_label->setStyleSheet(QStringLiteral(""));

        horizontalLayout->addWidget(user_label);

        userContainer = new QWidget(user_Widget);
        userContainer->setObjectName(QStringLiteral("userContainer"));
        gridLayout_7 = new QGridLayout(userContainer);
        gridLayout_7->setSpacing(0);
        gridLayout_7->setContentsMargins(11, 11, 11, 11);
        gridLayout_7->setObjectName(QStringLiteral("gridLayout_7"));
        gridLayout_7->setContentsMargins(5, 5, 5, 5);
        user_input = new QLineEdit(userContainer);
        user_input->setObjectName(QStringLiteral("user_input"));
        sizePolicy2.setHeightForWidth(user_input->sizePolicy().hasHeightForWidth());
        user_input->setSizePolicy(sizePolicy2);

        gridLayout_7->addWidget(user_input, 0, 0, 1, 1);


        horizontalLayout->addWidget(userContainer);

        horizontalLayout->setStretch(0, 1);
        horizontalLayout->setStretch(1, 3);

        verticalLayout->addWidget(user_Widget);

        verticalSpacer_5 = new QSpacerItem(20, 15, QSizePolicy::Minimum, QSizePolicy::Fixed);

        verticalLayout->addItem(verticalSpacer_5);

        pass_Widget = new QWidget(formpanel);
        pass_Widget->setObjectName(QStringLiteral("pass_Widget"));
        horizontalLayout_2 = new QHBoxLayout(pass_Widget);
        horizontalLayout_2->setSpacing(20);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        passwd_label = new QLabel(pass_Widget);
        passwd_label->setObjectName(QStringLiteral("passwd_label"));
        passwd_label->setFont(font);
        passwd_label->setStyleSheet(QStringLiteral(""));

        horizontalLayout_2->addWidget(passwd_label);

        passContainer = new QWidget(pass_Widget);
        passContainer->setObjectName(QStringLiteral("passContainer"));
        gridLayout_8 = new QGridLayout(passContainer);
        gridLayout_8->setSpacing(0);
        gridLayout_8->setContentsMargins(11, 11, 11, 11);
        gridLayout_8->setObjectName(QStringLiteral("gridLayout_8"));
        gridLayout_8->setContentsMargins(5, 5, 5, 5);
        passwd_input = new QLineEdit(passContainer);
        passwd_input->setObjectName(QStringLiteral("passwd_input"));
        sizePolicy2.setHeightForWidth(passwd_input->sizePolicy().hasHeightForWidth());
        passwd_input->setSizePolicy(sizePolicy2);

        gridLayout_8->addWidget(passwd_input, 0, 0, 1, 1);


        horizontalLayout_2->addWidget(passContainer);

        horizontalLayout_2->setStretch(0, 1);
        horizontalLayout_2->setStretch(1, 3);

        verticalLayout->addWidget(pass_Widget);

        verticalSpacer_9 = new QSpacerItem(20, 15, QSizePolicy::Minimum, QSizePolicy::Fixed);

        verticalLayout->addItem(verticalSpacer_9);

        widget_3 = new QWidget(formpanel);
        widget_3->setObjectName(QStringLiteral("widget_3"));
        horizontalLayout_6 = new QHBoxLayout(widget_3);
        horizontalLayout_6->setSpacing(20);
        horizontalLayout_6->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        horizontalLayout_6->setContentsMargins(0, 0, 0, 0);
        passwd_label_2 = new QLabel(widget_3);
        passwd_label_2->setObjectName(QStringLiteral("passwd_label_2"));
        passwd_label_2->setFont(font);
        passwd_label_2->setStyleSheet(QStringLiteral(""));

        horizontalLayout_6->addWidget(passwd_label_2);

        trustedGB = new QGroupBox(widget_3);
        trustedGB->setObjectName(QStringLiteral("trustedGB"));
        horizontalLayout_5 = new QHBoxLayout(trustedGB);
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        yesRB = new QRadioButton(trustedGB);
        yesRB->setObjectName(QStringLiteral("yesRB"));
        QFont font2;
        font2.setPointSize(12);
        yesRB->setFont(font2);
        yesRB->setStyleSheet(QStringLiteral(""));

        horizontalLayout_5->addWidget(yesRB);

        noRB = new QRadioButton(trustedGB);
        noRB->setObjectName(QStringLiteral("noRB"));
        noRB->setFont(font2);
        noRB->setStyleSheet(QStringLiteral(""));
        noRB->setChecked(true);

        horizontalLayout_5->addWidget(noRB);


        horizontalLayout_6->addWidget(trustedGB);

        horizontalLayout_6->setStretch(0, 1);
        horizontalLayout_6->setStretch(1, 3);

        verticalLayout->addWidget(widget_3);

        verticalSpacer_6 = new QSpacerItem(20, 30, QSizePolicy::Minimum, QSizePolicy::Fixed);

        verticalLayout->addItem(verticalSpacer_6);

        widget = new QWidget(formpanel);
        widget->setObjectName(QStringLiteral("widget"));
        sizePolicy1.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy1);
        widget->setBaseSize(QSize(0, 70));
        horizontalLayout_4 = new QHBoxLayout(widget);
        horizontalLayout_4->setSpacing(20);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Preferred, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer);

        widget_2 = new QWidget(widget);
        widget_2->setObjectName(QStringLiteral("widget_2"));
        gridLayout_3 = new QGridLayout(widget_2);
        gridLayout_3->setSpacing(0);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(20, 5, 5, 5);
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_3->addItem(horizontalSpacer_2, 0, 1, 1, 1);

        reset_button = new QPushButton(widget_2);
        reset_button->setObjectName(QStringLiteral("reset_button"));
        sizePolicy1.setHeightForWidth(reset_button->sizePolicy().hasHeightForWidth());
        reset_button->setSizePolicy(sizePolicy1);

        gridLayout_3->addWidget(reset_button, 0, 2, 1, 1);

        connect_button = new QPushButton(widget_2);
        connect_button->setObjectName(QStringLiteral("connect_button"));
        sizePolicy1.setHeightForWidth(connect_button->sizePolicy().hasHeightForWidth());
        connect_button->setSizePolicy(sizePolicy1);

        gridLayout_3->addWidget(connect_button, 0, 0, 1, 1);

        gridLayout_3->setColumnStretch(0, 10);
        gridLayout_3->setColumnStretch(1, 1);
        gridLayout_3->setColumnStretch(2, 10);

        horizontalLayout_4->addWidget(widget_2);

        horizontalLayout_4->setStretch(0, 1);
        horizontalLayout_4->setStretch(1, 3);

        verticalLayout->addWidget(widget);


        gridLayout_4->addWidget(formpanel, 1, 0, 1, 1);

        gridLayout_4->setRowStretch(1, 1);

        gridLayout_2->addLayout(gridLayout_4, 0, 0, 1, 1);

        container->addWidget(containerPage1);
        containerPage2 = new QWidget();
        containerPage2->setObjectName(QStringLiteral("containerPage2"));
        containerPage2->setStyleSheet(QStringLiteral(""));
        gridLayout_15 = new QGridLayout(containerPage2);
        gridLayout_15->setSpacing(6);
        gridLayout_15->setContentsMargins(11, 11, 11, 11);
        gridLayout_15->setObjectName(QStringLiteral("gridLayout_15"));
        gridLayout_15->setContentsMargins(0, 5, 0, 5);
        gridLayout_9 = new QGridLayout();
        gridLayout_9->setSpacing(6);
        gridLayout_9->setObjectName(QStringLiteral("gridLayout_9"));
        gridLayout_9->setContentsMargins(0, 0, 0, 0);
        formpanel_2 = new QWidget(containerPage2);
        formpanel_2->setObjectName(QStringLiteral("formpanel_2"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(formpanel_2->sizePolicy().hasHeightForWidth());
        formpanel_2->setSizePolicy(sizePolicy3);
        formpanel_2->setStyleSheet(QStringLiteral(""));
        verticalLayout_2 = new QVBoxLayout(formpanel_2);
        verticalLayout_2->setSpacing(0);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(20, 10, 20, 20);
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        horizontalLayout_8->setContentsMargins(0, -1, -1, -1);
        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer_4);

        closeButton_3 = new QPushButton(formpanel_2);
        closeButton_3->setObjectName(QStringLiteral("closeButton_3"));
        sizePolicy.setHeightForWidth(closeButton_3->sizePolicy().hasHeightForWidth());
        closeButton_3->setSizePolicy(sizePolicy);
        closeButton_3->setMinimumSize(QSize(20, 20));
        closeButton_3->setStyleSheet(QLatin1String("#closeButton_3{background-color: transparent;\n"
"font: Bold 14pt \"Leelawadee\";\n"
"color:red;}\n"
"#closeButton_3:hover{background-color: red;\n"
"font: Bold 14pt \"Leelawadee\";\n"
"color:white;}"));

        horizontalLayout_8->addWidget(closeButton_3, 0, Qt::AlignTop);


        verticalLayout_2->addLayout(horizontalLayout_8);

        dbname_Widget_2 = new QWidget(formpanel_2);
        dbname_Widget_2->setObjectName(QStringLiteral("dbname_Widget_2"));
        passWidget_2 = new QHBoxLayout(dbname_Widget_2);
        passWidget_2->setSpacing(20);
        passWidget_2->setContentsMargins(11, 11, 11, 11);
        passWidget_2->setObjectName(QStringLiteral("passWidget_2"));
        passWidget_2->setContentsMargins(0, 0, 0, 0);
        servenameContainer_2 = new QWidget(dbname_Widget_2);
        servenameContainer_2->setObjectName(QStringLiteral("servenameContainer_2"));
        sizePolicy2.setHeightForWidth(servenameContainer_2->sizePolicy().hasHeightForWidth());
        servenameContainer_2->setSizePolicy(sizePolicy2);
        servenameContainer_2->setMinimumSize(QSize(0, 150));
        gridLayout_10 = new QGridLayout(servenameContainer_2);
        gridLayout_10->setSpacing(0);
        gridLayout_10->setContentsMargins(11, 11, 11, 11);
        gridLayout_10->setObjectName(QStringLiteral("gridLayout_10"));
        gridLayout_10->setContentsMargins(5, 5, 5, 5);
        dbPathLineEdit = new QLineEdit(servenameContainer_2);
        dbPathLineEdit->setObjectName(QStringLiteral("dbPathLineEdit"));
        sizePolicy2.setHeightForWidth(dbPathLineEdit->sizePolicy().hasHeightForWidth());
        dbPathLineEdit->setSizePolicy(sizePolicy2);
        dbPathLineEdit->setMinimumSize(QSize(0, 50));
        dbPathLineEdit->setFont(font1);
        dbPathLineEdit->setAlignment(Qt::AlignJustify|Qt::AlignVCenter);

        gridLayout_10->addWidget(dbPathLineEdit, 1, 0, 1, 1);

        server_label_2 = new QLabel(servenameContainer_2);
        server_label_2->setObjectName(QStringLiteral("server_label_2"));
        sizePolicy1.setHeightForWidth(server_label_2->sizePolicy().hasHeightForWidth());
        server_label_2->setSizePolicy(sizePolicy1);
        server_label_2->setFont(font);
        server_label_2->setStyleSheet(QStringLiteral(""));

        gridLayout_10->addWidget(server_label_2, 0, 0, 1, 1);


        passWidget_2->addWidget(servenameContainer_2);


        verticalLayout_2->addWidget(dbname_Widget_2);

        widget_5 = new QWidget(formpanel_2);
        widget_5->setObjectName(QStringLiteral("widget_5"));
        sizePolicy1.setHeightForWidth(widget_5->sizePolicy().hasHeightForWidth());
        widget_5->setSizePolicy(sizePolicy1);
        widget_5->setBaseSize(QSize(0, 70));
        horizontalLayout_12 = new QHBoxLayout(widget_5);
        horizontalLayout_12->setSpacing(20);
        horizontalLayout_12->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_12->setObjectName(QStringLiteral("horizontalLayout_12"));
        horizontalLayout_12->setContentsMargins(0, 0, 0, 0);
        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Preferred, QSizePolicy::Minimum);

        horizontalLayout_12->addItem(horizontalSpacer_5);

        widget_6 = new QWidget(widget_5);
        widget_6->setObjectName(QStringLiteral("widget_6"));
        gridLayout_14 = new QGridLayout(widget_6);
        gridLayout_14->setSpacing(0);
        gridLayout_14->setContentsMargins(11, 11, 11, 11);
        gridLayout_14->setObjectName(QStringLiteral("gridLayout_14"));
        gridLayout_14->setContentsMargins(20, 5, 5, 5);
        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_14->addItem(horizontalSpacer_6, 0, 1, 1, 1);

        reset_button_2 = new QPushButton(widget_6);
        reset_button_2->setObjectName(QStringLiteral("reset_button_2"));
        sizePolicy1.setHeightForWidth(reset_button_2->sizePolicy().hasHeightForWidth());
        reset_button_2->setSizePolicy(sizePolicy1);
        reset_button_2->setMinimumSize(QSize(0, 50));

        gridLayout_14->addWidget(reset_button_2, 0, 2, 1, 1);

        connect_button_2 = new QPushButton(widget_6);
        connect_button_2->setObjectName(QStringLiteral("connect_button_2"));
        sizePolicy2.setHeightForWidth(connect_button_2->sizePolicy().hasHeightForWidth());
        connect_button_2->setSizePolicy(sizePolicy2);
        connect_button_2->setMinimumSize(QSize(0, 50));

        gridLayout_14->addWidget(connect_button_2, 0, 0, 1, 1);

        gridLayout_14->setColumnStretch(0, 10);
        gridLayout_14->setColumnStretch(1, 1);
        gridLayout_14->setColumnStretch(2, 10);

        horizontalLayout_12->addWidget(widget_6);

        horizontalLayout_12->setStretch(0, 1);
        horizontalLayout_12->setStretch(1, 3);

        verticalLayout_2->addWidget(widget_5);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_2);

        verticalLayout_2->setStretch(1, 1);
        verticalLayout_2->setStretch(2, 1);

        gridLayout_9->addWidget(formpanel_2, 0, 0, 1, 1);


        gridLayout_15->addLayout(gridLayout_9, 1, 0, 1, 1);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_15->addItem(verticalSpacer_3, 0, 0, 1, 1);

        verticalSpacer_10 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_15->addItem(verticalSpacer_10, 2, 0, 1, 1);

        container->addWidget(containerPage2);

        gridLayout->addWidget(container, 2, 0, 1, 1);


        retranslateUi(DbDialog);

        container->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(DbDialog);
    } // setupUi

    void retranslateUi(QDialog *DbDialog)
    {
        DbDialog->setWindowTitle(QApplication::translate("DbDialog", "DbDialog", Q_NULLPTR));
        closeButton_2->setText(QApplication::translate("DbDialog", "X", Q_NULLPTR));
        server_label->setText(QApplication::translate("DbDialog", "Server Name:", Q_NULLPTR));
        server_input->setText(QString());
        db_label->setText(QApplication::translate("DbDialog", "DB Name:     ", Q_NULLPTR));
        user_label->setText(QApplication::translate("DbDialog", "DB User:       ", Q_NULLPTR));
        passwd_label->setText(QApplication::translate("DbDialog", "Password:     ", Q_NULLPTR));
        passwd_label_2->setText(QApplication::translate("DbDialog", "Trusted:             ", Q_NULLPTR));
        trustedGB->setTitle(QString());
        yesRB->setText(QApplication::translate("DbDialog", "Yes", Q_NULLPTR));
        noRB->setText(QApplication::translate("DbDialog", "No", Q_NULLPTR));
        reset_button->setText(QApplication::translate("DbDialog", "RESET", Q_NULLPTR));
        connect_button->setText(QApplication::translate("DbDialog", "SET PARAMETERS", Q_NULLPTR));
        closeButton_3->setText(QApplication::translate("DbDialog", "X", Q_NULLPTR));
        dbPathLineEdit->setText(QString());
        server_label_2->setText(QApplication::translate("DbDialog", "Db Path:", Q_NULLPTR));
        reset_button_2->setText(QApplication::translate("DbDialog", "RESET", Q_NULLPTR));
        connect_button_2->setText(QApplication::translate("DbDialog", "SET PARAMETERS", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class DbDialog: public Ui_DbDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DBDIALOG_H
