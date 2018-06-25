/********************************************************************************
** Form generated from reading UI file 'TurtleButler.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TURTLEBUTLER_H
#define UI_TURTLEBUTLER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TurtleButlerWidget
{
public:
    QWidget *centralWidget;
    QPushButton *pushButton;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QComboBox *robot_comboBox;
    QComboBox *origin_comboBox;
    QComboBox *destination_comboBox;
    QPlainTextEdit *item_textEdit;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_5;
    QLabel *robot_label;
    QLabel *origin_label;
    QLabel *destination_label;
    QLabel *item_label;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *TurtleButlerWidget)
    {
        if (TurtleButlerWidget->objectName().isEmpty())
            TurtleButlerWidget->setObjectName(QStringLiteral("TurtleButlerWidget"));
        TurtleButlerWidget->resize(451, 335);
        centralWidget = new QWidget(TurtleButlerWidget);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(180, 240, 89, 25));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(160, 10, 261, 201));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        robot_comboBox = new QComboBox(verticalLayoutWidget);
        robot_comboBox->setObjectName(QStringLiteral("robot_comboBox"));
        robot_comboBox->setMaximumSize(QSize(16777215, 30));

        verticalLayout->addWidget(robot_comboBox);

        origin_comboBox = new QComboBox(verticalLayoutWidget);
        origin_comboBox->setObjectName(QStringLiteral("origin_comboBox"));
        origin_comboBox->setMaximumSize(QSize(16777215, 30));

        verticalLayout->addWidget(origin_comboBox);

        destination_comboBox = new QComboBox(verticalLayoutWidget);
        destination_comboBox->setObjectName(QStringLiteral("destination_comboBox"));
        destination_comboBox->setMaximumSize(QSize(16777215, 30));

        verticalLayout->addWidget(destination_comboBox);

        item_textEdit = new QPlainTextEdit(verticalLayoutWidget);
        item_textEdit->setObjectName(QStringLiteral("item_textEdit"));
        item_textEdit->setMaximumSize(QSize(16777215, 30));

        verticalLayout->addWidget(item_textEdit);

        verticalLayoutWidget_2 = new QWidget(centralWidget);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(20, 10, 121, 201));
        verticalLayout_5 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        robot_label = new QLabel(verticalLayoutWidget_2);
        robot_label->setObjectName(QStringLiteral("robot_label"));
        robot_label->setMaximumSize(QSize(16777215, 30));
        robot_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_5->addWidget(robot_label);

        origin_label = new QLabel(verticalLayoutWidget_2);
        origin_label->setObjectName(QStringLiteral("origin_label"));
        origin_label->setMaximumSize(QSize(16777215, 30));
        origin_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_5->addWidget(origin_label);

        destination_label = new QLabel(verticalLayoutWidget_2);
        destination_label->setObjectName(QStringLiteral("destination_label"));
        destination_label->setMaximumSize(QSize(16777215, 30));
        destination_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_5->addWidget(destination_label);

        item_label = new QLabel(verticalLayoutWidget_2);
        item_label->setObjectName(QStringLiteral("item_label"));
        item_label->setMaximumSize(QSize(16777215, 30));
        item_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_5->addWidget(item_label);

        TurtleButlerWidget->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(TurtleButlerWidget);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 451, 22));
        TurtleButlerWidget->setMenuBar(menuBar);
        mainToolBar = new QToolBar(TurtleButlerWidget);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        TurtleButlerWidget->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(TurtleButlerWidget);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        TurtleButlerWidget->setStatusBar(statusBar);

        retranslateUi(TurtleButlerWidget);

        QMetaObject::connectSlotsByName(TurtleButlerWidget);
    } // setupUi

    void retranslateUi(QMainWindow *TurtleButlerWidget)
    {
        TurtleButlerWidget->setWindowTitle(QApplication::translate("TurtleButlerWidget", "MainWindow", 0));
        pushButton->setText(QApplication::translate("TurtleButlerWidget", "Send", 0));
        robot_label->setText(QApplication::translate("TurtleButlerWidget", "Robot", 0));
        origin_label->setText(QApplication::translate("TurtleButlerWidget", "Origin", 0));
        destination_label->setText(QApplication::translate("TurtleButlerWidget", "Destination", 0));
        item_label->setText(QApplication::translate("TurtleButlerWidget", "Item", 0));
    } // retranslateUi

};

namespace Ui {
    class TurtleButlerWidget: public Ui_TurtleButlerWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TURTLEBUTLER_H
